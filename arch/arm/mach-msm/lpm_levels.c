/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <mach/mpm.h>
#include "lpm_resources.h"
#include "pm.h"
#include "rpm-notifier.h"

static struct msm_rpmrs_level *msm_lpm_levels;
static int msm_lpm_level_count;

enum {
	MSM_LPM_LVL_DBG_SUSPEND_LIMITS = BIT(0),
	MSM_LPM_LVL_DBG_IDLE_LIMITS = BIT(1),
};

struct power_params {
	uint32_t latency_us;
	uint32_t ss_power;
	uint32_t energy_overhead;
	uint32_t time_overhead_us;
	uint32_t target_residency_us;
};

struct lpm_cpu_level {
	const char *name;
	enum msm_pm_sleep_mode mode;
	struct power_params pwr;
	bool use_bc_timer;
	bool sync;
};

struct lpm_system_level {
	const char *name;
	uint32_t l2_mode;
	struct power_params pwr;
	enum msm_pm_sleep_mode min_cpu_mode;
	int num_cpu_votes;
	bool notify_rpm;
	bool available;
	bool sync;
};

struct lpm_system_state {
	struct lpm_cpu_level *cpu_level;
	int num_cpu_levels;
	struct lpm_system_level *system_level;
	int num_system_levels;
	enum msm_pm_sleep_mode sync_cpu_mode;
	int last_entered_cluster_index;
	bool allow_synched_levels;
	bool no_l2_saw;
	struct spinlock sync_lock;
	int num_cores_in_sync;
};

static struct lpm_system_state sys_state;
static bool suspend_in_progress;
static int64_t suspend_time;

struct lpm_lookup_table {
	uint32_t modes;
	const char *mode_name;
};

static void lpm_system_level_update(void);
static void setup_broadcast_timer(void *arg);
static int lpm_cpu_callback(struct notifier_block *cpu_nb,
				unsigned long action, void *hcpu);

static struct notifier_block __refdata lpm_cpu_nblk = {
	.notifier_call = lpm_cpu_callback,
};

static uint32_t allowed_l2_mode;
static uint32_t sysfs_dbg_l2_mode __refdata = MSM_SPM_L2_MODE_POWER_COLLAPSE;
static uint32_t default_l2_mode;


static ssize_t lpm_levels_attr_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t lpm_levels_attr_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count);


static int lpm_lvl_dbg_msk;

module_param_named(
	debug_mask, lpm_lvl_dbg_msk, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static bool menu_select;
module_param_named(
	menu_select, menu_select, bool, S_IRUGO | S_IWUSR | S_IWGRP
);

static int msm_pm_sleep_time_override;
module_param_named(sleep_time_override,
	msm_pm_sleep_time_override, int, S_IRUGO | S_IWUSR | S_IWGRP);

static int num_powered_cores;
static struct hrtimer lpm_hrtimer;

static struct kobj_attribute lpm_l2_kattr = __ATTR(l2,  S_IRUGO|S_IWUSR,\
		lpm_levels_attr_show, lpm_levels_attr_store);

static struct attribute *lpm_levels_attr[] = {
	&lpm_l2_kattr.attr,
	NULL,
};

static struct attribute_group lpm_levels_attr_grp = {
	.attrs = lpm_levels_attr,
};

/* SYSFS */
static ssize_t lpm_levels_attr_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned int lpm_level;
	struct msm_rpmrs_level *level = NULL;

	for (lpm_level = 0; lpm_level < msm_lpm_level_count; lpm_level++) {
		level = &msm_lpm_levels[lpm_level];
		level->available =
			!msm_lpm_level_beyond_limit(&level->rs_limits);
	}
}

int msm_lpm_enter_sleep(uint32_t sclk_count, void *limits,
		bool from_idle, bool notify_rpm)
{
	int ret = 0;
	struct msm_rpmrs_limits *l = (struct msm_rpmrs_limits *)limits;

	ret = msm_rpm_enter_sleep();
	if (ret) {
		pr_warn("%s(): RPM failed to enter sleep err:%d\n",
				__func__, ret);
		goto bail;
	}
	ret = msm_lpmrs_enter_sleep(sclk_count, l, from_idle, notify_rpm);
bail:
	return ret;
}

static void msm_lpm_exit_sleep(void *limits, bool from_idle,
		bool notify_rpm, bool collapsed)
{
	msm_rpm_exit_sleep();
	msm_lpmrs_exit_sleep((struct msm_rpmrs_limits *)limits,
				from_idle, notify_rpm, collapsed);
	int lpm = sleep_mode;
	int rc = 0;

	if (system_state->no_l2_saw)
		goto bail_set_l2_mode;

	msm_pm_set_l2_flush_flag(MSM_SCM_L2_ON);

	switch (sleep_mode) {
	case MSM_SPM_L2_MODE_POWER_COLLAPSE:
		pr_info("Configuring for L2 power collapse\n");
		msm_pm_set_l2_flush_flag(MSM_SCM_L2_OFF);
		break;
	case MSM_SPM_L2_MODE_GDHS:
		msm_pm_set_l2_flush_flag(MSM_SCM_L2_GDHS);
		break;
	case MSM_SPM_L2_MODE_PC_NO_RPM:
		msm_pm_set_l2_flush_flag(MSM_SCM_L2_OFF);
		break;
	case MSM_SPM_L2_MODE_RETENTION:
	case MSM_SPM_L2_MODE_DISABLED:
		break;
	default:
		lpm = MSM_SPM_L2_MODE_DISABLED;
		break;
	}

	rc = msm_spm_l2_set_low_power_mode(lpm, true);

	if (rc) {
		if (rc == -ENXIO)
			WARN_ON_ONCE(1);
		else
			pr_err("%s: Failed to set L2 low power mode %d, ERR %d",
					__func__, lpm, rc);
	}
bail_set_l2_mode:
	return rc;
}

void msm_lpm_show_resources(void)
{
	/* TODO */
	return;
}

s32 msm_cpuidle_get_deep_idle_latency(void)
{
	int i;
	struct msm_rpmrs_level *level = msm_lpm_levels, *best = level;

	if (!level)
		return 0;

	for (i = 0; i < msm_lpm_level_count; i++, level++) {
		if (!level->available)
			continue;
		if (level->sleep_mode != MSM_PM_SLEEP_MODE_POWER_COLLAPSE)
			continue;
		/* Pick the first power collapse mode by default */
		if (best->sleep_mode != MSM_PM_SLEEP_MODE_POWER_COLLAPSE)
			best = level;
		/* Find the lowest latency for power collapse */
		if (level->latency_us < best->latency_us)
			best = level;

		if ((sleep_us >> 10) > pwr_param->time_overhead_us) {
			pwr = pwr_param->ss_power;
		} else {
			pwr = pwr_param->ss_power;
			pwr -= (pwr_param->time_overhead_us
					* pwr_param->ss_power) / sleep_us;
			pwr += pwr_param->energy_overhead / sleep_us;
		}

		if (best_level_pwr >= pwr) {
			best_level = i;
			best_level_pwr = pwr;
		}
	}
	return best_level;
}

static void lpm_system_prepare(struct lpm_system_state *system_state,
		int index, bool from_idle)
{
	struct lpm_system_level *lvl;
	struct clock_event_device *bc = tick_get_broadcast_device()->evtdev;
	uint32_t sclk;
	int64_t us = (~0ULL);
	int dbg_mask;
	int ret;
	const struct cpumask *nextcpu;

	spin_lock(&system_state->sync_lock);
	if (num_powered_cores != system_state->num_cores_in_sync) {
		spin_unlock(&system_state->sync_lock);
		return;
	}

	if (from_idle) {
		dbg_mask = lpm_lvl_dbg_msk & MSM_LPM_LVL_DBG_IDLE_LIMITS;
		us = ktime_to_us(ktime_sub(bc->next_event, ktime_get()));
		nextcpu = bc->cpumask;
	} else {
		dbg_mask = lpm_lvl_dbg_msk & MSM_LPM_LVL_DBG_SUSPEND_LIMITS;
		nextcpu = cpumask_of(smp_processor_id());
	}

	lvl = &system_state->system_level[index];

	ret = lpm_set_l2_mode(system_state, lvl->l2_mode);

	if (ret && ret != -ENXIO) {
		pr_warn("%s(): Cannot set L2 Mode %d, ret:%d\n",
				__func__, lvl->l2_mode, ret);
		goto bail_system_sleep;
	}

	if (lvl->notify_rpm) {
		ret = msm_rpm_enter_sleep(dbg_mask, nextcpu);
		if (ret) {
			pr_err("rpm_enter_sleep() failed with rc = %d\n", ret);
			goto bail_system_sleep;
		}


		if (!from_idle)
			us = USEC_PER_SEC * msm_pm_sleep_time_override;

		do_div(us, USEC_PER_SEC/SCLK_HZ);
		sclk = (uint32_t)us;
		msm_mpm_enter_sleep(sclk, from_idle, nextcpu);
	}
	return best->latency_us - 1;
}
static bool msm_lpm_irqs_detectable(struct msm_rpmrs_limits *limits,
		bool irqs_detectable, bool gpio_detectable)
{
	if (!limits->irqs_detectable)
		return irqs_detectable;

	if (!limits->gpio_detectable)
		return gpio_detectable;

	return true;

}

static void *msm_lpm_lowest_limits(bool from_idle,
		enum msm_pm_sleep_mode sleep_mode,
		struct msm_pm_time_params *time_param, uint32_t *power)
{
	unsigned int cpu = smp_processor_id();
	struct msm_rpmrs_level *best_level = NULL;
	uint32_t pwr;
	int i;
	int best_level_iter = msm_lpm_level_count + 1;
	bool irqs_detect = false;
	bool gpio_detect = false;

	if (!msm_lpm_levels)
		return NULL;

	msm_lpm_level_update();

	if (sleep_mode == MSM_PM_SLEEP_MODE_POWER_COLLAPSE) {
		irqs_detect = msm_mpm_irqs_detectable(from_idle);
		gpio_detect = msm_mpm_gpio_irqs_detectable(from_idle);
	}

	for (i = 0; i < msm_lpm_level_count; i++) {
		struct msm_rpmrs_level *level = &msm_lpm_levels[i];

		if (!level->available)
			continue;

		if (sleep_mode != level->sleep_mode)
			continue;

		if (time_param->latency_us < level->latency_us)
			continue;

		if ((sleep_mode == MSM_PM_SLEEP_MODE_POWER_COLLAPSE) &&
			!msm_lpm_irqs_detectable(&level->rs_limits,
				irqs_detect, gpio_detect))
				continue;

		if ((MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE == sleep_mode)
			|| (MSM_PM_SLEEP_MODE_POWER_COLLAPSE == sleep_mode))
			if (!cpu && msm_rpm_waiting_for_ack())
					break;

		if (time_param->sleep_us <= 1) {
			pwr = level->energy_overhead;
		} else if (time_param->sleep_us <= level->time_overhead_us) {
			pwr = level->energy_overhead / time_param->sleep_us;
		} else if ((time_param->sleep_us >> 10)
				> level->time_overhead_us) {
			pwr = level->steady_state_power;
		} else {
			pwr = level->steady_state_power;
			pwr -= (level->time_overhead_us *
				level->steady_state_power) /
						time_param->sleep_us;
			pwr += level->energy_overhead / time_param->sleep_us;
		}

		if (!best_level || best_level->rs_limits.power[cpu] >= pwr) {

			level->rs_limits.latency_us[cpu] = level->latency_us;
			level->rs_limits.power[cpu] = pwr;
			best_level = level;

			if (power)
				*power = pwr;
		}
	}

	return best_level ? &best_level->rs_limits : NULL;
}
static struct msm_pm_sleep_ops msm_lpm_ops = {
	.lowest_limits = msm_lpm_lowest_limits,
	.enter_sleep = msm_lpm_enter_sleep,
	.exit_sleep = msm_lpm_exit_sleep,
};

static int __devinit msm_lpm_levels_probe(struct platform_device *pdev)
{
	struct msm_rpmrs_level *levels = NULL;
	struct msm_rpmrs_level *level = NULL;
	struct device_node *node = NULL;
	char *key = NULL;
	uint32_t val = 0;
	int ret = 0;
	uint32_t num_levels = 0;
	int idx = 0;

	for_each_child_of_node(pdev->dev.of_node, node)
		num_levels++;

	levels = kzalloc(num_levels * sizeof(struct msm_rpmrs_level),
			GFP_KERNEL);
	if (!levels)
		return -ENOMEM;

	for_each_child_of_node(pdev->dev.of_node, node) {
		level = &levels[idx++];
		level->available = false;

		key = "qcom,mode";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->sleep_mode = val;

		key = "qcom,xo";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->rs_limits.pxo = val;

		key = "qcom,l2";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->rs_limits.l2_cache = val;

		key = "qcom,vdd-dig-upper-bound";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->rs_limits.vdd_dig_upper_bound = val;

		key = "qcom,vdd-dig-lower-bound";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->rs_limits.vdd_dig_lower_bound = val;

		key = "qcom,vdd-mem-upper-bound";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->rs_limits.vdd_mem_upper_bound = val;

		key = "qcom,vdd-mem-lower-bound";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->rs_limits.vdd_mem_lower_bound = val;

		key = "qcom,latency-us";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->latency_us = val;

		key = "qcom,ss-power";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->steady_state_power = val;

		key = "qcom,energy-overhead";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->energy_overhead = val;

		key = "qcom,time-overhead";
		ret = of_property_read_u32(node, key, &val);
		if (ret)
			goto fail;
		level->time_overhead_us = val;

		level->available = true;
	}

	msm_lpm_levels = levels;
	msm_lpm_level_count = idx;

	msm_pm_set_sleep_ops(&msm_lpm_ops);

	return 0;
fail:
	pr_err("%s: Error in name %s key %s\n", __func__, node->full_name, key);
	kfree(levels);
	return -EFAULT;
}

<<<<<<< HEAD
static struct of_device_id msm_lpm_levels_match_table[] = {
=======
static struct of_device_id cpu_modes_mtch_tbl[] __initdata = {
	{.compatible = "qcom,cpu-modes"},
	{},
};

static struct platform_driver cpu_modes_driver = {
	.probe = lpm_cpu_probe,
	.driver = {
		.name = "cpu-modes",
		.owner = THIS_MODULE,
		.of_match_table = cpu_modes_mtch_tbl,
	},
};

static struct of_device_id system_modes_mtch_tbl[] __initdata = {
	{.compatible = "qcom,system-modes"},
	{},
};

static struct platform_driver system_modes_driver = {
	.probe = lpm_system_probe,
	.driver = {
		.name = "system-modes",
		.owner = THIS_MODULE,
		.of_match_table = system_modes_mtch_tbl,
	},
};

static struct of_device_id lpm_levels_match_table[] __initdata = {
>>>>>>> 1ec0b659970... ARM: msm: GCC Version change.
	{.compatible = "qcom,lpm-levels"},
	{},
};

static struct platform_driver msm_lpm_levels_driver = {
	.probe = msm_lpm_levels_probe,
	.driver = {
		.name = "lpm-levels",
		.owner = THIS_MODULE,
		.of_match_table = msm_lpm_levels_match_table,
	},
};

static int __init msm_lpm_levels_module_init(void)
{
	return platform_driver_register(&msm_lpm_levels_driver);
}
late_initcall(msm_lpm_levels_module_init);

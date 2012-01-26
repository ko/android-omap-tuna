/*
 *  linux/drivers/cpufreq/cpufreq_scaffold.c
 *
 *  Copyright (C) 2002 - 2003 Dominik Brodowski <linux@brodo.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/init.h>


static int cpufreq_governor_scaffold(struct cpufreq_policy *policy,
					unsigned int event)
{
	switch (event) {
	case CPUFREQ_GOV_START:
	case CPUFREQ_GOV_LIMITS:
		pr_debug("setting to %u kHz because of event %u\n",
						policy->max, event);
		__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
		break;
	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_CPU_FREQ_GOV_SCAFFOLD_MODULE
static
#endif
struct cpufreq_governor cpufreq_gov_scaffold = {
	.name		= "scaffold",
	.governor	= cpufreq_governor_scaffold,
	.owner		= THIS_MODULE,
};


static int __init cpufreq_gov_scaffold_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_scaffold);
}


static void __exit cpufreq_gov_scaffold_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_scaffold);
}


MODULE_AUTHOR("Kenneth Ko <ko@yaksok.net>");
MODULE_DESCRIPTION("CPUfreq policy governor 'scaffold'");
MODULE_LICENSE("GPL");

fs_initcall(cpufreq_gov_scaffold_init);
module_exit(cpufreq_gov_scaffold_exit);

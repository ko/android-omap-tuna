/*
 * drivers/cpufreq/cpufreq_scaffold.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Kenneth Ko <ko@yaksok.net>
 *
 * Based off the interactive governor by Mike Chan (mike@android.com) included
 * in the mainline tuna kernel as of Jan. 2012
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/mutex.h>

#include <asm/cputime.h>

static atomic_t active_count = ATOMIC_INIT(0);

struct cpufreq_scaffold_cpuinfo {
	struct timer_list cpu_timer;
	int timer_idlecancel;
	u64 time_in_idle;
	u64 idle_exit_time;
	u64 timer_run_time;
	u64 freq_change_time;
	u64 freq_change_time_in_idle;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	unsigned int target_freq;
	int governor_enabled;
    /* NEW */
    int cur_cpu_load;
    unsigned int force_ramp_up;
    int max_speed;
    int min_speed;
	int idling;
    int suspended;
};

static DEFINE_PER_CPU(struct cpufreq_scaffold_cpuinfo, cpuinfo);

/* Workqueues handle frequency scaling */
static struct workqueue_struct *up_wq;
static struct workqueue_struct *down_wq;
static struct work_struct freq_scale_work;
static cpumask_t up_cpumask;
static spinlock_t up_cpumask_lock;
static cpumask_t down_cpumask;
static spinlock_t down_cpumask_lock;
static struct mutex set_speed_lock;

enum {
        SCAFFOLD_DEBUG_JUMPS=1,
        SCAFFOLD_DEBUG_LOAD=2,
        SCAFFOLD_DEBUG_IDLE=4
};

/* 
 * Combination of the above debug flags.
 */
static unsigned long debug_mask = 7;

/* Hi speed to bump to from lo speed when load burst (default max) */
static u64 hispeed_freq;

/* Go to hi speed when CPU load at or above this value. */
#define DEFAULT_GO_HISPEED_LOAD 90
static unsigned long go_hispeed_load;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 */
#define DEFAULT_MIN_SAMPLE_TIME 10 * USEC_PER_MSEC
static unsigned long min_sample_time;

/*
 * The sample rate of the timer used to increase frequency
 */
#define DEFAULT_TIMER_RATE 20 * USEC_PER_MSEC
static unsigned long timer_rate;

static int cpufreq_governor_scaffold(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_scaffold
static
#endif
struct cpufreq_governor cpufreq_gov_scaffold = {
	.name = "scaffold",
	.governor = cpufreq_governor_scaffold,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

static void cpufreq_scaffold_timer(unsigned long data)
{
	unsigned int delta_idle;
	unsigned int delta_time;
	int cpu_load;
	int load_since_change;
	u64 time_in_idle;
	u64 idle_exit_time;
	struct cpufreq_scaffold_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	u64 now_idle;
	unsigned int new_freq;
	unsigned int index;
	unsigned long flags;

	smp_rmb();

	if (!pcpu->governor_enabled)
		goto exit;

	/*
	 * Once pcpu->timer_run_time is updated to >= pcpu->idle_exit_time,
	 * this lets idle exit know the current idle time sample has
	 * been processed, and idle exit can generate a new sample and
	 * re-arm the timer.  This prevents a concurrent idle
	 * exit on that CPU from writing a new set of info at the same time
	 * the timer function runs (the timer function can't use that info
	 * until more time passes).
	 */
	time_in_idle = pcpu->time_in_idle;
	idle_exit_time = pcpu->idle_exit_time;
	now_idle = get_cpu_idle_time_us(data, &pcpu->timer_run_time);
	smp_wmb();

	/* If we raced with cancelling a timer, skip. */
	if (!idle_exit_time)
		goto exit;

	delta_idle = (unsigned int) cputime64_sub(now_idle, time_in_idle);
	delta_time = (unsigned int) cputime64_sub(pcpu->timer_run_time,
						  idle_exit_time);

	/*
	 * If timer ran less than 1ms after short-term sample started, retry.
	 */
	if (delta_time < 1000)
		goto rearm;

	if (delta_idle > delta_time)
		cpu_load = 0;
	else
		cpu_load = 100 * (delta_time - delta_idle) / delta_time;

	delta_idle = (unsigned int) cputime64_sub(now_idle,
						pcpu->freq_change_time_in_idle);
	delta_time = (unsigned int) cputime64_sub(pcpu->timer_run_time,
						  pcpu->freq_change_time);

	if ((delta_time == 0) || (delta_idle > delta_time))
		load_since_change = 0;
	else
		load_since_change =
			100 * (delta_time - delta_idle) / delta_time;

	/*
	 * Choose greater of short-term load (since last idle timer
	 * started or timer function re-armed itself) or long-term load
	 * (since last frequency change).
	 */
	if (load_since_change > cpu_load)
		cpu_load = load_since_change;

	if (cpu_load >= go_hispeed_load) {
		if (pcpu->policy->cur == pcpu->policy->min)
			new_freq = hispeed_freq;
		else
			new_freq = pcpu->policy->max * cpu_load / 100;
	} else {
		new_freq = pcpu->policy->cur * cpu_load / 100;
	}

	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
					   new_freq, CPUFREQ_RELATION_H,
					   &index)) {
		pr_warn_once("timer %d: cpufreq_frequency_table_target error\n",
			     (int) data);
		goto rearm;
	}

	new_freq = pcpu->freq_table[index].frequency;

	if (pcpu->target_freq == new_freq)
		goto rearm_if_notmax;

	/*
	 * Do not scale down unless we have been at this frequency for the
	 * minimum sample time.
	 */
	if (new_freq < pcpu->target_freq) {
		if (cputime64_sub(pcpu->timer_run_time, pcpu->freq_change_time)
		    < min_sample_time)
			goto rearm;
	}

	if (new_freq < pcpu->target_freq) {
		pcpu->target_freq = new_freq;
		spin_lock_irqsave(&down_cpumask_lock, flags);
        /* TODO down_cpumask
         */
		cpumask_set_cpu(data, &down_cpumask);
		spin_unlock_irqrestore(&down_cpumask_lock, flags);
		queue_work(down_wq, &freq_scale_work);
	} else {
		pcpu->target_freq = new_freq;
		spin_lock_irqsave(&up_cpumask_lock, flags);
		cpumask_set_cpu(data, &up_cpumask);
		spin_unlock_irqrestore(&up_cpumask_lock, flags);
		wake_up_process(up_task);
	}

rearm_if_notmax:
	/*
	 * Already set max speed and don't see a need to change that,
	 * wait until next idle to re-evaluate, don't need timer.
	 */
	if (pcpu->target_freq == pcpu->policy->max)
		goto exit;

rearm:
	if (!timer_pending(&pcpu->cpu_timer)) {
		/*
		 * If already at min: if that CPU is idle, don't set timer.
		 * Else cancel the timer if that CPU goes idle.  We don't
		 * need to re-evaluate speed until the next idle exit.
		 */
		if (pcpu->target_freq == pcpu->policy->min) {
			smp_rmb();

			if (pcpu->idling)
				goto exit;

			pcpu->timer_idlecancel = 1;
		}

		pcpu->time_in_idle = get_cpu_idle_time_us(
			data, &pcpu->idle_exit_time);
		mod_timer(&pcpu->cpu_timer,
			  jiffies + usecs_to_jiffies(timer_rate));
	}

exit:
	return;
}

static void cpufreq_scaffold_idle_start(void)
{
	struct cpufreq_scaffold_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	int pending;

	if (!pcpu->governor_enabled)
		return;

	pcpu->idling = 1;
	smp_wmb();
	pending = timer_pending(&pcpu->cpu_timer);

	if (pcpu->target_freq != pcpu->policy->min) {
#ifdef CONFIG_SMP
		/*
		 * Entering idle while not at lowest speed.  On some
		 * platforms this can hold the other CPU(s) at that speed
		 * even though the CPU is idle. Set a timer to re-evaluate
		 * speed so this idle CPU doesn't hold the other CPUs above
		 * min indefinitely.  This should probably be a quirk of
		 * the CPUFreq driver.
		 */
		if (!pending) {
			pcpu->time_in_idle = get_cpu_idle_time_us(
				smp_processor_id(), &pcpu->idle_exit_time);
			pcpu->timer_idlecancel = 0;
			mod_timer(&pcpu->cpu_timer,
				  jiffies + usecs_to_jiffies(timer_rate));
		}
#endif
	} else {
		/*
		 * If at min speed and entering idle after load has
		 * already been evaluated, and a timer has been set just in
		 * case the CPU suddenly goes busy, cancel that timer.  The
		 * CPU didn't go busy; we'll recheck things upon idle exit.
		 */
		if (pending && pcpu->timer_idlecancel) {
			del_timer(&pcpu->cpu_timer);
			/*
			 * Ensure last timer run time is after current idle
			 * sample start time, so next idle exit will always
			 * start a new idle sampling period.
			 */
			pcpu->idle_exit_time = 0;
			pcpu->timer_idlecancel = 0;
		}
	}

}

static void cpufreq_scaffold_idle_end(void)
{
	struct cpufreq_scaffold_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

	pcpu->idling = 0;
	smp_wmb();

	/*
	 * Arm the timer for 1-2 ticks later if not already, and if the timer
	 * function has already processed the previous load sampling
	 * interval.  (If the timer is not pending but has not processed
	 * the previous interval, it is probably racing with us on another
	 * CPU.  Let it compute load based on the previous sample and then
	 * re-arm the timer for another interval when it's done, rather
	 * than updating the interval start time to be "now", which doesn't
	 * give the timer function enough time to make a decision on this
	 * run.)
	 */
	if (timer_pending(&pcpu->cpu_timer) == 0 &&
	    pcpu->timer_run_time >= pcpu->idle_exit_time &&
	    pcpu->governor_enabled) {
		pcpu->time_in_idle =
			get_cpu_idle_time_us(smp_processor_id(),
					     &pcpu->idle_exit_time);
		pcpu->timer_idlecancel = 0;
		mod_timer(&pcpu->cpu_timer,
			  jiffies + usecs_to_jiffies(timer_rate));
	}

}

static int cpufreq_scaffold_up_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_scaffold_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&up_cpumask_lock, flags);

		if (cpumask_empty(&up_cpumask)) {
			spin_unlock_irqrestore(&up_cpumask_lock, flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&up_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = up_cpumask;
		cpumask_clear(&up_cpumask);
		spin_unlock_irqrestore(&up_cpumask_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			unsigned int j;
			unsigned int max_freq = 0;

			pcpu = &per_cpu(cpuinfo, cpu);
			smp_rmb();

			if (!pcpu->governor_enabled)
				continue;

			mutex_lock(&set_speed_lock);

			for_each_cpu(j, pcpu->policy->cpus) {
				struct cpufreq_scaffold_cpuinfo *pjcpu =
					&per_cpu(cpuinfo, j);

				if (pjcpu->target_freq > max_freq)
					max_freq = pjcpu->target_freq;
			}

			if (max_freq != pcpu->policy->cur)
				__cpufreq_driver_target(pcpu->policy,
							max_freq,
							CPUFREQ_RELATION_H);
			mutex_unlock(&set_speed_lock);

			pcpu->freq_change_time_in_idle =
				get_cpu_idle_time_us(cpu,
						     &pcpu->freq_change_time);
		}
	}

	return 0;
}

/* Workqueue handler function that was started at __init
 */
static void cpufreq_scaffold_freq_change_time_work(struct work_struct *work)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_scaffold_cpuinfo *pcpu;

    /* Save current state of local interrupts,
     * disables local interrupts,
     * acquires lock
     */
	spin_lock_irqsave(&down_cpumask_lock, flags);
	tmp_mask = down_cpumask;
	cpumask_clear(&down_cpumask);
	spin_unlock_irqrestore(&down_cpumask_lock, flags);

    /* for ((cpu) = 0; (cpu) < 1; (cpu)++, (void)mask)
     */
	for_each_cpu(cpu, &tmp_mask) {
		unsigned int j;
		unsigned int max_freq = 0;

		pcpu = &per_cpu(cpuinfo, cpu);
        /* commit all pending loads before continuing
         * thanks
         */
		smp_rmb();

		if (!pcpu->governor_enabled)
			continue;

		mutex_lock(&set_speed_lock);

        /* for (j = 0; j < 1; j++, (void)(pcpu->policy->cpus))
         */
		for_each_cpu(j, pcpu->policy->cpus) {
			struct cpufreq_scaffold_cpuinfo *pjcpu =
				&per_cpu(cpuinfo, j);

			if (pjcpu->target_freq > max_freq)
				max_freq = pjcpu->target_freq;
		}

		if (max_freq != pcpu->policy->cur)
			__cpufreq_driver_target(pcpu->policy, max_freq,
						CPUFREQ_RELATION_H);

		mutex_unlock(&set_speed_lock);
		pcpu->freq_change_time_in_idle =
			get_cpu_idle_time_us(cpu,
					     &pcpu->freq_change_time);
	}
}

static ssize_t show_hispeed_freq(struct kobject *kobj,
				 struct attribute *attr, char *buf)
{
	return sprintf(buf, "%llu\n", hispeed_freq);
}

static ssize_t store_hispeed_freq(struct kobject *kobj,
				  struct attribute *attr, const char *buf,
				  size_t count)
{
	int ret;
	u64 val;

	ret = strict_strtoull(buf, 0, &val);
	if (ret < 0)
		return ret;
	hispeed_freq = val;
	return count;
}

static struct global_attr hispeed_freq_attr = __ATTR(hispeed_freq, 0644,
		show_hispeed_freq, store_hispeed_freq);


static ssize_t show_go_hispeed_load(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", go_hispeed_load);
}

static ssize_t store_go_hispeed_load(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	go_hispeed_load = val;
	return count;
}

static struct global_attr go_hispeed_load_attr = __ATTR(go_hispeed_load, 0644,
		show_go_hispeed_load, store_go_hispeed_load);

static ssize_t show_min_sample_time(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", min_sample_time);
}

static ssize_t store_min_sample_time(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	min_sample_time = val;
	return count;
}

static struct global_attr min_sample_time_attr = __ATTR(min_sample_time, 0644,
		show_min_sample_time, store_min_sample_time);

static ssize_t show_timer_rate(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", timer_rate);
}

static ssize_t store_timer_rate(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	timer_rate = val;
	return count;
}

static struct global_attr timer_rate_attr = __ATTR(timer_rate, 0644,
		show_timer_rate, store_timer_rate);

static struct attribute *scaffold_attributes[] = {
	&hispeed_freq_attr.attr,
	&go_hispeed_load_attr.attr,
	&min_sample_time_attr.attr,
	&timer_rate_attr.attr,
	NULL,
};

static struct attribute_group scaffold_attr_group = {
	.attrs = scaffold_attributes,
	.name = "scaffold",
};

/* This is the governor function which we registered in __init
 */
static int cpufreq_governor_scaffold(struct cpufreq_policy *policy,
		unsigned int event)
{
	int rc;
	unsigned int j;
	struct cpufreq_scaffold_cpuinfo *pcpu;
	struct cpufreq_frequency_table *freq_table;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(policy->cpu))
			return -EINVAL;

        /* Array of struct consisting of (index, frequency)
         */
		freq_table =
			cpufreq_frequency_get_table(policy->cpu);

		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			pcpu->freq_change_time_in_idle =
				get_cpu_idle_time_us(j,
					     &pcpu->freq_change_time);
			pcpu->governor_enabled = 1;
            /* flush pending store instructions now before moving on
             */
			smp_wmb();
		}

		if (!hispeed_freq)
			hispeed_freq = policy->max;

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (atomic_inc_return(&active_count) > 1)
			return 0;

        /* given a directory kobject, create an attribute group
         *
         * struct attribute_group {
         *    const char *name
         *    umode_t (*is_visible)(struct kobject *, struct attribute *, int);
         *    struct attribute **attrs
         * }
         */
		rc = sysfs_create_group(cpufreq_global_kobject,
				&scaffold_attr_group);
		if (rc)
			return rc;

		break;

	case CPUFREQ_GOV_STOP:
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->governor_enabled = 0;
			smp_wmb();
			del_timer_sync(&pcpu->cpu_timer);

			/*
			 * Reset idle exit time since we may cancel the timer
			 * before it can run after the last idle exit time,
			 * to avoid tripping the check in idle exit for a timer
			 * that is trying to run.
			 */
			pcpu->idle_exit_time = 0;
		}

		flush_work(&freq_scale_work);
		if (atomic_dec_return(&active_count) > 0)
			return 0;

		sysfs_remove_group(cpufreq_global_kobject,
				&scaffold_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
		break;
	}
	return 0;
}

static void cpufreq_scaffold_suspend(int cpu, int suspend)
{
        struct scaffold_info_s *pcpu = &per_cpu(scaffold_info, 
                                                smp_process_id()); 
        struct cpufreq_policy *policy = pcpu->cur_policy;
        unsigned int new_freq;

        if (!pcpu->enable || sleep_max_freq == 0)
                // disable behaviour for (sleep_max_freq == 0)
                return;
        scaffold_update_min_max(pcpu, policy, suspend);

        pcpu->suspended = suspend;

        if (!suspend) {
                // resume at max speed
                new_freq = validate_freq(pcpu, sleep_wakeup_freq);

                if (debug_mask & SCAFFOLD_DEBUG_JUMPS)
                        printk(KERN_INFO "%s: waking up at %d\n", 
                                __FUNCTION__, new_freq);

                // TODO: CPUFREQ_RELATION_L == ?
                __cpufreq_driver_target(policy, new_freq, CPUFREQ_RELATION_L);

                if (policy->cur < pcpu->max_speed 
                        && !timer_pending(&pcpu->timer)) {
                        reset_timer(smp_processor_id(), pcpu, 
                                    sample_rate_jiffies);
                } else {
                        /* to avoid wakeup issues with quick sleep/wakeup, don't
                         * change actual frequency when entering sleep to allow
                         * some time to settle down. We reset the timer, if 
                         * eventually, even at full load the timer the timer
                         * will lower the frequency
                         */
                        reset_timer(smp_processor_id(), pcpu, 
                                sample_rate_jiffies);

                        pcpu->freq_change_time_in_idle = 
                                get_cpu_idle_time_us(
                                        cpu, &pcpu->freq_change_time);

                        if (debug_mask & SMARTASS_DEBUG_JUMPS)
                                printk(KERN_INFO "%s: suspending at %d\n",
                                        __FUNCTION__, policy->cur);
                }
        }

}

static int cpufreq_scaffold_idle_notifier(struct notifier_block *nb,
					     unsigned long val,
					     void *data)
{
	switch (val) {
	case IDLE_START:
		cpufreq_scaffold_idle_start();
		break;
	case IDLE_END:
		cpufreq_scaffold_idle_end();
		break;
	}

	return 0;
}

static struct notifier_block cpufreq_scaffold_idle_nb = {
        .notifier_call = cpufreq_scaffold_idle_notifier,
};

static void scaffold_early_suspend(struct early_suspend *handler) {
        int i;
        for_each_online_cpu(i)
                cpufreq_scaffold_suspend(i,1);
}
static void scaffold_late_resume(struct early_suspend *handler) {
        int i;
        for_each_online_cpu(i)
                cpufreq_scaffold_suspend(i,0);
}

static struct early_suspend cpufreq_scaffold_suspend_nb = {
        /* TODO: need these functions */
        .suspend = scaffold_early_suspend,
        .resume = scaffold_late_resume,
        .level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};

/*
 * The init.
 */
static int __init cpufreq_scaffold_init(void)
{
	unsigned int i;
	struct cpufreq_scaffold_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 }; 

	go_hispeed_load = DEFAULT_GO_HISPEED_LOAD;
	min_sample_time = DEFAULT_MIN_SAMPLE_TIME;
	timer_rate = DEFAULT_TIMER_RATE;

	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
        // Maintain a per-cpu area prevents cache line bouncing 
		pcpu = &per_cpu(cpuinfo, i);

        // don't wake up the kernel just for this
		init_timer_deferrable(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_scaffold_timer;
		pcpu->cpu_timer.data = i;
	}

    up_wq = alloc_ordered_workqueue("kscaffold_up", WQ_MEM_RECLAIM);
    if (!up_wq)
        goto err_freeuptask;

	/* No rescuer thread, bind to CPU queuing the work for possibly
	   warm cache (probably doesn't matter much). 
       
       We don't use create_workqueue because it "has no dedicated threads
       and really just serves as a context for the submission of tasks"
       [http://lwn.net/Articles/403891/]
     */
	down_wq = alloc_workqueue("kscaffold_down", 0, 1);

	if (!down_wq)
		goto err_freeuptask;

	INIT_WORK(&freq_scale_work, cpufreq_scaffold_freq_change_time_work);

    /* Adding this from smartass port.
     * TODO look into this
     */
    register_early_suspend(&scaffold_power_suspend);

    printk(KERN_INFO "scaffold: loaded\n");

    /* TODO do we need these
     */
	spin_lock_init(&up_cpumask_lock);
	spin_lock_init(&down_cpumask_lock);
	mutex_init(&set_speed_lock);

    /* Is this deprecated? 
     * TODO figure it out
	idle_notifier_register(&cpufreq_scaffold_idle_nb);
     */
    register_idle_notifier(&cpufreq_scaffold_idle_nb);

	return cpufreq_register_governor(&cpufreq_gov_scaffold);

err_freeuptask:
    destroy_workqueue(up_wq);
	return -ENOMEM;
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_scaffold
fs_initcall(cpufreq_scaffold_init);
#else
module_init(cpufreq_scaffold_init);
#endif

static void __exit cpufreq_scaffold_exit(void)
{
    /* TODO what?
     */
    struct scaffold_info_s *pcpu = &per_cpu(scaffold_info, 0);
    pcpu->enable = 0;

	cpufreq_unregister_governor(&cpufreq_gov_scaffold);
    unregister_early_suspend(&scaffold_power_suspend);

    destroy_workqueue(up_wq);
	destroy_workqueue(down_wq);
}

module_exit(cpufreq_scaffold_exit);

MODULE_AUTHOR("Kenneth Ko <ko@yaksok.net>");
MODULE_DESCRIPTION("'cpufreq_scaffold' - A cpufreq governor for "
	"scaffolding");
MODULE_LICENSE("GPL");

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

#include <linux/earlysuspend.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>

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
static cpumask_t work_cpumask;
static unsigned int suspended;

enum {
        SCAFFOLD_DEBUG_JUMPS=1,
        SCAFFOLD_DEBUG_LOAD=2,
        SCAFFOLD_DEBUG_IDLE=4,
        SCAFFOLD_DEBUG_GENERAL=8,
};

/* Combination of the above debug flags. */
#define DEFAULT_DEBUG_MASK 0;
static unsigned long debug_mask;

/* The minimum amount of time to spend at a frequency before we can ramp up. */
#define DEFAULT_UP_RATE_US 24000;
static unsigned long up_rate_us;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 */
#define DEFAULT_DOWN_RATE_US 24000;
static unsigned long down_rate_us;

/*
 * When ramping up frequency with no idle cycles jump to at least this frequency.
 * Zero disables. Set a very high value to jump to policy max freqeuncy.
 */
#define DEFAULT_UP_MIN_FREQ 1200000
static unsigned int up_min_freq;

/*
 * When sleep_max_freq>0 the frequency when suspended will be capped
 * by this frequency. Also will wake up at max frequency of policy
 * to minimize wakeup issues.
 * Set sleep_max_freq=0 to disable this behavior.
 */
#define DEFAULT_SLEEP_MAX_FREQ 245760
static unsigned int sleep_max_freq;

/*
 * The frequency to set when waking up from sleep.
 * When sleep_max_freq=0 this will have no effect.
 */
#define DEFAULT_SLEEP_WAKEUP_FREQ 998400 
static unsigned int sleep_wakeup_freq;

/*
 * When awake_min_freq>0 the frequency when not suspended will not
 * go below this frequency.
 * Set awake_min_freq=0 to disable this behavior.
 */
#define DEFAULT_AWAKE_MIN_FREQ 0
static unsigned int awake_min_freq;

/* Sampling rate, I highly recommend to leave it at 2. */
#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;

/*
 * Freqeuncy delta when ramping up.
 * zero disables and causes to always jump straight to max frequency.
 */
#define DEFAULT_RAMP_UP_STEP 350000
static unsigned int ramp_up_step;

/*
 * Freqeuncy delta when ramping down.
 * zero disables and will calculate ramp down according to load heuristic.
 */
#define DEFAULT_RAMP_DOWN_STEP 160000
static unsigned int ramp_down_step;

/*
 * CPU freq will be increased if measured load > max_cpu_load;
 */
#define DEFAULT_MAX_CPU_LOAD 75
static unsigned long max_cpu_load;

/*
 * CPU freq will be decreased if measured load < min_cpu_load;
 */
#define DEFAULT_MIN_CPU_LOAD 25
static unsigned long min_cpu_load;

/*
 * When screen if off behave like conservative governor;
 */
#define DEFAULT_SLEEP_RATE_US (usecs_to_jiffies(500000))
static unsigned long sleep_rate_jiffies;

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

static void scaffold_update_min_max(struct cpufreq_scaffold_cpuinfo *pcpu, 
                                    struct cpufreq_policy *policy, int suspend) 
{
        if (suspend) {
                pcpu->min_speed = policy->min;
                pcpu->max_speed = // sleep_max_freq; but make sure 
                                  // it obeys the policy min/max
                        policy->max > sleep_max_freq ? 
                                (sleep_max_freq > policy->min ? 
                                         sleep_max_freq : policy->min) 
                                : policy->max;
        } else {
                pcpu->min_speed = // awake_min_freq; but make sure 
                                  // it obeys the policy min/max
                        policy->min < awake_min_freq ? 
                                (awake_min_freq < policy->max ? 
                                        awake_min_freq : policy->max) 
                                : policy->min;
                pcpu->max_speed = policy->max;
        }
}
inline static unsigned int validate_freq(struct cpufreq_scaffold_cpuinfo *pcpu, 
                                        int freq) 
{
        if (freq > pcpu->max_speed)
                return pcpu->max_speed;
        if (freq < pcpu->min_speed)
                return pcpu->min_speed;
        return freq;
}
static void reset_timer(unsigned long cpu, 
                        struct cpufreq_scaffold_cpuinfo *pcpu, 
                        unsigned int timer_rate_jiffies) 
{
        pcpu->time_in_idle = get_cpu_idle_time_us(cpu, &pcpu->idle_exit_time);
        mod_timer(&pcpu->cpu_timer, jiffies + timer_rate_jiffies);
}
static void cpufreq_scaffold_timer(unsigned long data)
{
        unsigned int delta_idle;
        unsigned int delta_time;
        int cpu_load;
        u64 idle_exit_time;
        u64 time_in_idle;
        u64 now_idle;
        struct cpufreq_scaffold_cpuinfo *pcpu = &per_cpu(cpuinfo, data);
        struct cpufreq_policy *policy = pcpu->policy;
        unsigned int timer_rate_jiffies = sample_rate_jiffies;

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

        /* Screen is off so we can be more relaxed. */
        if (pcpu->suspended && policy->cur == policy->min)
            timer_rate_jiffies = sleep_rate_jiffies;

        delta_idle = (unsigned int) cputime64_sub(now_idle, time_in_idle);
        delta_time = (unsigned int) cputime64_sub(pcpu->timer_run_time,
                              idle_exit_time);

        if (debug_mask & SCAFFOLD_DEBUG_IDLE)
            printk(KERN_INFO "%s: t=%u i=%u\n",
                    __FUNCTION__, delta_time, delta_idle);

        /*
         * If timer ran less than 1ms after short-term sample started, retry.
         */
        if (delta_time < 1000)
            goto rearm;

        if (delta_idle > delta_time)
            cpu_load = 0;
        else
            cpu_load = 100 * (delta_time - delta_idle) / delta_time;

        if (debug_mask & SCAFFOLD_DEBUG_LOAD)
            printk(KERN_INFO "%s:%d: cur=%d, calc'dload=%d, delta_time=%u)\n",
                    __FUNCTION__, __LINE__, policy->cur, cpu_load, delta_time);

        pcpu->cur_cpu_load = cpu_load;

        /* Scale up if load > max || if there were no idle cycles since
         * coming out of idle || we are above our max speed for a very
         * long time (should only happen if entering sleep at high loads)
         */
        if (cpu_load > max_cpu_load || delta_idle == 0) {
                if (!(policy->cur > pcpu->max_speed &&
                        cputime64_sub(pcpu->timer_run_time,
                                pcpu->freq_change_time)
                        > 100 * down_rate_us)) {
                        
                    if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                        printk(KERN_INFO "%s:%d: load=%d, max=%d, diff=%u\n",
                                __FUNCTION__, __LINE__, cpu_load, max_cpu_load,
                                (unsigned int)cputime64_sub(
                                            pcpu->timer_run_time, 
                                            pcpu->freq_change_time));

                    if (policy->cur > pcpu->max_speed)
                            reset_timer(data, pcpu, timer_rate_jiffies);

                    if (policy->cur == policy->max)
                            goto exit;

                    if (cputime64_sub(pcpu->timer_run_time, 
                                    pcpu->freq_change_time) < up_rate_us) {

                            if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                                printk(KERN_INFO "%s:%d: diff pre-exit: %u\n",
                                        __FUNCTION__, __LINE__, (unsigned int)
                                        cputime64_sub(pcpu->timer_run_time,
                                                    pcpu->freq_change_time)
                                );

                            goto exit;
                    }

                    pcpu->force_ramp_up = 1;
                    cpumask_set_cpu(data, &work_cpumask);
                    queue_work(up_wq, &freq_scale_work);
                    goto exit;
                }
        }

        if (policy->cur < pcpu->max_speed && !timer_pending(&pcpu->cpu_timer)) {
            if (policy->cur == policy->min)
                    if (pcpu->idling)
                            goto exit;

            reset_timer(data, pcpu, timer_rate_jiffies);
        }
            
        /* Do not scale down unless we have been at this frequency
         * for the minimum sample time.
         */
        if (cputime64_sub(pcpu->timer_run_time, pcpu->freq_change_time)
                < min_sample_time) {

                goto exit;
        }

        if (cputime64_sub(pcpu->timer_run_time, pcpu->freq_change_time)
                < down_rate_us) {

                if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                        printk("%s: timer_run_time=%u, freq_change_time=%u\n",
                                __FUNCTION__, 
                                (unsigned int)pcpu->timer_run_time, 
                                (unsigned int)pcpu->freq_change_time);
                goto exit;
        }

        /* Scale down only when there is something to scale down.
         */
        if (cpu_load < min_cpu_load) {
                if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                        printk(KERN_INFO "%s:%d: cpu_load(%d) < "
                                "min_cpu_load(%u)\n",
                                __FUNCTION__, __LINE__, cpu_load, 
                                (unsigned int)min_cpu_load);
                cpumask_set_cpu(data, &work_cpumask);
                queue_work(down_wq, &freq_scale_work);
        }
        /* There is a window where if the cpu utilization can go from
         * low to high between the timer expiring, delta_idle will be
         * > 0 and the cpu will be 100% busy, preventing idle from
         * running, and this timer from firing. So setup another timer
         * to fire to check cpu utilization. Do not setup the timer
         * if there is no scheduled work or if at max speed.
         */
rearm:
        if (!timer_pending(&pcpu->cpu_timer)) {
                /* If already at min and cpu is idle, don't
                 * set the timer. Else cancel the timer if
                 * the CPU goes idle. We don't need to re-evaluate
                 * speed until the next idle exit.
                 */

                if (policy->cur < pcpu->max_speed) {
                        if (policy->cur == policy->min) {
                                if (pcpu->idling)
                                        goto exit;
                                pcpu->timer_idlecancel = 1;
                        }
                        if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                            printk(KERN_INFO "policy->cur < pcpu->max_speed\n");
                        reset_timer(data, pcpu, timer_rate_jiffies);
                }
        }
exit:
	return;
}

static void cpufreq_scaffold_idle_start(void)
{
	struct cpufreq_scaffold_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
    struct cpufreq_policy *policy = pcpu->policy;
	int pending;
    unsigned int timer_rate_jiffies = sample_rate_jiffies;

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
        if (!timer_pending(&pcpu->cpu_timer)) {
            if (pcpu->suspended && policy->cur == policy->min)
                    timer_rate_jiffies = sleep_rate_jiffies;
            reset_timer(smp_processor_id(), pcpu, timer_rate_jiffies);
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

/* We use the same work function to scale up and down */
static void cpufreq_scaffold_freq_change_time_work(struct work_struct *work)
{
        unsigned int cpu;
        int freq_new;
        unsigned force_ramp_up;
        int cpu_load;
        struct cpufreq_scaffold_cpuinfo *pcpu;
        struct cpufreq_policy *policy; 
        unsigned int relation = CPUFREQ_RELATION_L;
        cpumask_t tmp_mask = work_cpumask;
        const cpumask_t * ptmp_mask = &tmp_mask;
        unsigned int index;

        for_each_cpu(cpu, ptmp_mask) {
                pcpu = &per_cpu(cpuinfo, cpu);
                policy = pcpu->policy;
                cpu_load = pcpu->cur_cpu_load;
                force_ramp_up = pcpu->force_ramp_up;
                pcpu->force_ramp_up = 0;

                smp_rmb();

                if (!pcpu->governor_enabled)
                    continue;

                if (force_ramp_up || cpu_load > max_cpu_load) {
                        if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                                printk(KERN_INFO 
                                        "%s: %d, force_ramp_up=%u, "
                                        "cpu_load=%d\n",
                                        __FUNCTION__, __LINE__, force_ramp_up,
                                        cpu_load);

                        if (force_ramp_up && up_min_freq) {
                                freq_new = up_min_freq;
                                relation = CPUFREQ_RELATION_L;
                                if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                                        printk(KERN_INFO 
                                                "%s:%d: force_up=%d, "
                                                "min_freq=%d\n",
                                                __FUNCTION__, __LINE__, 
                                                force_ramp_up,
                                                up_min_freq);
                        } else if (ramp_up_step) {
                                freq_new = policy->cur + ramp_up_step;
                                relation = CPUFREQ_RELATION_H;
                                if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                                        printk(KERN_INFO
                                                "%s:%d: ramp_up_step=%d",
                                                __FUNCTION__, __LINE__, 
                                                ramp_up_step);
                        } else {
                                freq_new = pcpu->max_speed;
                                relation = CPUFREQ_RELATION_H;
                                if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                                        printk(KERN_INFO
                                                "%s:%d: up + else "
                                                "freq_new=%d\n",
                                                __FUNCTION__, __LINE__, 
                                                freq_new);
                        }
                } else if (cpu_load < min_cpu_load) {
                        if (ramp_down_step)
                                freq_new = policy->cur - ramp_down_step;
                        else {
                                // TODO why in the hell?
                                cpu_load += 100 - max_cpu_load; // dummy load ?
                                freq_new = policy->cur * cpu_load / 100;
                                if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                                        printk(KERN_INFO "%s: freq_new: %d\n",
                                                __FUNCTION__, freq_new);
                        } 
                        relation = CPUFREQ_RELATION_L;
                } else {
                        freq_new = policy->cur;
                        if (debug_mask & SCAFFOLD_DEBUG_GENERAL)
                                printk(KERN_INFO "%s:%d: !up + else "
                                        "freq_new=%d\n",
                                        __FUNCTION__, __LINE__, freq_new);
                }

                freq_new = validate_freq(pcpu, freq_new);


                /* TODO CPUFREQ_RELATION_H ? */
                if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
                            freq_new, relation, &index)) {
                        pr_warn_once("%s:%d: "
                                "timer: cpufreq_frequency_table_target_error\n",
                                __FUNCTION__, __LINE__);
                        // TODO get out ?
                }

                freq_new= pcpu->freq_table[index].frequency;

                if (freq_new != policy->cur) {
                        if (debug_mask & SCAFFOLD_DEBUG_JUMPS)
                                printk(KERN_INFO "%s: jumping from %d to %d\n",
                                        __FUNCTION__, policy->cur, freq_new);

                        __cpufreq_driver_target(policy, freq_new, relation);

                        pcpu->freq_change_time_in_idle = 
                                get_cpu_idle_time_us(cpu,
                                        &pcpu->freq_change_time);
                }

                cpumask_clear_cpu(cpu, &work_cpumask);
        }
}

/*
 * Mutators for debug_mask
 */
static ssize_t show_debug_mask(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%lu\n", debug_mask);
}

static ssize_t 
store_debug_mask(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0)
                debug_mask = input;
        return res;
}

static struct freq_attr debug_mask_attr = __ATTR(debug_mask, 0644,
        show_debug_mask, store_debug_mask);

/* Mutators for up_rate_us */
static ssize_t show_up_rate_us(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%lu\n", up_rate_us);
}
static ssize_t store_up_rate_us(struct cpufreq_policy *policy, 
                                const char *buf, ssize_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 0 && input <= 100000000)
                up_rate_us = input;
        return res;
}
static struct freq_attr up_rate_us_attr = __ATTR(up_rate_us, 0644,
        show_up_rate_us, store_up_rate_us);

/* Mutators for down_rate_us */
static ssize_t show_down_rate_us(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%lu\n", down_rate_us);
}
static ssize_t store_down_rate_us(struct cpufreq_policy *policy, 
                                const char *buf, ssize_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 0 && input <= 100000000)
                down_rate_us = input;
        return res;
}
static struct freq_attr down_rate_us_attr = __ATTR(down_rate_us, 0644,
                show_down_rate_us, store_down_rate_us);

/* Mutators for up_min_freq */
static ssize_t show_up_min_freq(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%u\n", up_min_freq);
}
static ssize_t store_up_min_freq(struct cpufreq_policy *policy, 
                                const char *buf, ssize_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 0)
          up_min_freq = input;
        return res;
}
static struct freq_attr up_min_freq_attr = __ATTR(up_min_freq, 0644,
                show_up_min_freq, store_up_min_freq);

/* Mutators for sleep_max_freq */
static ssize_t show_sleep_max_freq(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%u\n", sleep_max_freq);
}
static ssize_t store_sleep_max_freq(struct cpufreq_policy *policy, 
                                    const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 0)
          sleep_max_freq = input;
        return res;
}
static struct freq_attr sleep_max_freq_attr = __ATTR(sleep_max_freq, 0644,
                show_sleep_max_freq, store_sleep_max_freq);

/* Mutators for sleep_wakeup_freq */
static ssize_t show_sleep_wakeup_freq(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%u\n", sleep_wakeup_freq);
}
static ssize_t store_sleep_wakeup_freq(struct cpufreq_policy *policy, 
                                        const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 0)
          sleep_wakeup_freq = input;
        return res;
}
static struct freq_attr sleep_wakeup_freq_attr = __ATTR(sleep_wakeup_freq, 0644,
                show_sleep_wakeup_freq, store_sleep_wakeup_freq);

/* Mutators awake_min_freq */
static ssize_t show_awake_min_freq(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%u\n", awake_min_freq);
}
static ssize_t store_awake_min_freq(struct cpufreq_policy *policy, 
                                    const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 0)
          awake_min_freq = input;
        return res;
}
static struct freq_attr awake_min_freq_attr = __ATTR(awake_min_freq, 0644,
                show_awake_min_freq, store_awake_min_freq);

/* Mutators sample_rate_jiffies */
static ssize_t show_sample_rate_jiffies(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%u\n", sample_rate_jiffies);
}

static ssize_t store_sample_rate_jiffies(struct cpufreq_policy *policy, 
                                        const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input > 0 && input <= 1000)
          sample_rate_jiffies = input;
        return res;
}
static struct freq_attr sample_rate_jiffies_attr = __ATTR(sample_rate_jiffies, 0644,
                show_sample_rate_jiffies, store_sample_rate_jiffies);

/* Mutators ramp_up_step */
static ssize_t show_ramp_up_step(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%u\n", ramp_up_step);
}
static ssize_t store_ramp_up_step(struct cpufreq_policy *policy, 
                                const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 0)
          ramp_up_step = input;
        return res;
}
static struct freq_attr ramp_up_step_attr = __ATTR(ramp_up_step, 0644,
                show_ramp_up_step, store_ramp_up_step);

/* Mutators ramp_down_step */
static ssize_t show_ramp_down_step(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%u\n", ramp_down_step);
}
static ssize_t store_ramp_down_step(struct cpufreq_policy *policy, 
                                    const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 0)
          ramp_down_step = input;
        return res;
}
static struct freq_attr ramp_down_step_attr = __ATTR(ramp_down_step, 0644,
                show_ramp_down_step, store_ramp_down_step);

/* Mutators max_cpu_load */
static ssize_t show_max_cpu_load(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%lu\n", max_cpu_load);
}
static ssize_t store_max_cpu_load(struct cpufreq_policy *policy, 
                                const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input > 0 && input <= 100)
          max_cpu_load = input;
        return res;
}
static struct freq_attr max_cpu_load_attr = __ATTR(max_cpu_load, 0644,
                show_max_cpu_load, store_max_cpu_load);

/* Mutators min_cpu_load */
static ssize_t show_min_cpu_load(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%lu\n", min_cpu_load);
}
static ssize_t store_min_cpu_load(struct cpufreq_policy *policy, 
                                const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input > 0 && input < 100)
          min_cpu_load = input;
        return res;
}
static struct freq_attr min_cpu_load_attr = __ATTR(min_cpu_load, 0644,
                show_min_cpu_load, store_min_cpu_load);

/* Mutators sleep_rate_us */
static ssize_t show_sleep_rate_us(struct cpufreq_policy *policy, char *buf)
{
        return sprintf(buf, "%lu\n", (long)jiffies_to_usecs(sleep_rate_jiffies));
}
static ssize_t store_sleep_rate_us(struct cpufreq_policy *policy, 
                                const char *buf, size_t count)
{
        ssize_t res;
        unsigned long input;
        res = strict_strtoul(buf, 0, &input);
        if (res >= 0 && input >= 50000 && input <= 1000000)
          sleep_rate_jiffies = usecs_to_jiffies(input);
        return res;
}
static struct freq_attr sleep_rate_us_attr = __ATTR(sleep_rate_us, 0644,
                show_sleep_rate_us, store_sleep_rate_us);


/* Mutators for hispeed_freq */
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


/* Mutators for go_hispeed_load */
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

/* Mutators for min_sample_time */
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

/* Mutators for timer_rate */
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
    &debug_mask_attr.attr,
    &up_rate_us_attr.attr,
    &down_rate_us_attr.attr,
    &up_min_freq_attr.attr,
    &sleep_max_freq_attr.attr,
    &sleep_wakeup_freq_attr.attr,
    &awake_min_freq_attr.attr,
    &sample_rate_jiffies_attr.attr,
    &ramp_up_step_attr.attr,
    &ramp_down_step_attr.attr,
    &max_cpu_load_attr.attr,
    &min_cpu_load_attr.attr,
	&sleep_rate_us_attr.attr,
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
        struct cpufreq_scaffold_cpuinfo *pcpu = &per_cpu(cpuinfo, 
                                                smp_processor_id()); 
        struct cpufreq_policy *policy = pcpu->policy;
        unsigned int new_freq;

        if (!pcpu->governor_enabled || sleep_max_freq == 0)
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
                        && !timer_pending(&pcpu->cpu_timer)) {
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

                        if (debug_mask & SCAFFOLD_DEBUG_JUMPS)
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
        .level = 150 + 1
};

/*
 * The init.
 */
static int __init cpufreq_scaffold_init(void)
{
        unsigned int i;
        struct cpufreq_scaffold_cpuinfo *pcpu;

        debug_mask = DEFAULT_DEBUG_MASK;
        up_rate_us = DEFAULT_UP_RATE_US;
        down_rate_us = DEFAULT_DOWN_RATE_US;
        up_min_freq = DEFAULT_UP_MIN_FREQ;
        sleep_max_freq = DEFAULT_SLEEP_MAX_FREQ;
        sleep_wakeup_freq = DEFAULT_SLEEP_WAKEUP_FREQ;
        awake_min_freq = DEFAULT_AWAKE_MIN_FREQ;
        sample_rate_jiffies = DEFAULT_SAMPLE_RATE_JIFFIES;
        ramp_up_step = DEFAULT_RAMP_UP_STEP;
        ramp_down_step = DEFAULT_RAMP_DOWN_STEP;
        max_cpu_load = DEFAULT_MAX_CPU_LOAD;
        min_cpu_load = DEFAULT_MIN_CPU_LOAD;
        sleep_rate_jiffies = DEFAULT_SLEEP_RATE_US;

        suspended = 0;

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
        register_early_suspend(&cpufreq_scaffold_suspend_nb);

        printk(KERN_INFO "scaffold: loaded\n");

        /* Is this deprecated? 
         * TODO figure it out
        register_idle_notifier(&cpufreq_scaffold_idle_nb);
         */
        idle_notifier_register(&cpufreq_scaffold_idle_nb);

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
    struct cpufreq_scaffold_cpuinfo *pcpu = &per_cpu(cpuinfo, 0);
    pcpu->governor_enabled = 0;

	cpufreq_unregister_governor(&cpufreq_gov_scaffold);
    unregister_early_suspend(&cpufreq_scaffold_suspend_nb);

    destroy_workqueue(up_wq);
	destroy_workqueue(down_wq);
}

module_exit(cpufreq_scaffold_exit);

MODULE_AUTHOR("Kenneth Ko <ko@yaksok.net>");
MODULE_DESCRIPTION("'cpufreq_scaffold' - A cpufreq governor for "
	"scaffolding");
MODULE_LICENSE("GPL");

/* Copyright (c) 2014-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"core_ctl: " fmt

#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/syscore_ops.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched/core_ctl.h>

#include <trace/events/sched.h>
#include "sched.h"
#include "walt.h"

int core_ctl_set_boost(bool boost)
{
	unsigned int index = 0;
	struct cluster_data *cluster;
	unsigned long flags;
	int ret = 0;
	bool boost_state_changed = false;

	if (unlikely(!initialized))
		return 0;

	spin_lock_irqsave(&state_lock, flags);
	for_each_cluster(cluster, index) {
		if (boost) {
			boost_state_changed = !cluster->boost;
			++cluster->boost;
		} else {
			if (!cluster->boost) {
				ret = -EINVAL;
				break;
			} else {
				--cluster->boost;
				boost_state_changed = !cluster->boost;
			}
		}
	}
	spin_unlock_irqrestore(&state_lock, flags);

	if (boost_state_changed) {
		index = 0;
		for_each_cluster(cluster, index) {
			apply_need(cluster);
			trace_core_ctl_set_boost(cluster->boost, index, ret);
		}
	}

	return ret;
}
EXPORT_SYMBOL(core_ctl_set_boost);

void core_ctl_notifier_register(struct notifier_block *n)
{
	atomic_notifier_chain_register(&core_ctl_notifier, n);
}

void core_ctl_notifier_unregister(struct notifier_block *n)
{
	atomic_notifier_chain_unregister(&core_ctl_notifier, n);
}

static void core_ctl_call_notifier(void)
{
	struct core_ctl_notif_data ndata;
	struct notifier_block *nb;

	/*
	 * Don't bother querying the stats when the notifier
	 * chain is empty.
	 */
	rcu_read_lock();
	nb = rcu_dereference_raw(core_ctl_notifier.head);
	rcu_read_unlock();

	if (!nb)
		return;

	ndata.nr_big = last_nr_big;
	ndata.coloc_load_pct = walt_get_default_coloc_group_load();

	atomic_notifier_call_chain(&core_ctl_notifier, 0, &ndata);
}

void core_ctl_check(u64 window_start)
{
	int cpu;
	struct cpu_data *c;
	struct cluster_data *cluster;
	unsigned int index = 0;
	unsigned long flags;

	if (unlikely(!initialized))
		return;

	if (window_start == core_ctl_check_timestamp)
		return;

	core_ctl_check_timestamp = window_start;

	spin_lock_irqsave(&state_lock, flags);
	for_each_possible_cpu(cpu) {

		c = &per_cpu(cpu_state, cpu);
		cluster = c->cluster;

		if (!cluster || !cluster->inited)
			continue;

		c->busy = sched_get_cpu_util(cpu);
	}
	spin_unlock_irqrestore(&state_lock, flags);

	update_running_avg();

	for_each_cluster(cluster, index) {
		if (eval_need(cluster))
			wake_up_core_ctl_thread(cluster);
	}

	core_ctl_call_notifier();
}

static void move_cpu_lru(struct cpu_data *cpu_data)
{
	unsigned long flags;

	spin_lock_irqsave(&state_lock, flags);
	list_del(&cpu_data->sib);
	list_add_tail(&cpu_data->sib, &cpu_data->cluster->lru);
	spin_unlock_irqrestore(&state_lock, flags);
}

static void cpuset_next(struct cluster_data *cluster) { }

static bool should_we_isolate(int cpu, struct cluster_data *cluster)
{
	return true;
}

static void try_to_isolate(struct cluster_data *cluster, unsigned int need)
{
	struct cpu_data *c, *tmp;
	unsigned long flags;
	unsigned int num_cpus = cluster->num_cpus;
	unsigned int nr_isolated = 0;
	bool first_pass = cluster->nr_not_preferred_cpus;

	/*
	 * Protect against entry being removed (and added at tail) by other
	 * thread (hotplug).
	 */
	spin_lock_irqsave(&state_lock, flags);
	list_for_each_entry_safe(c, tmp, &cluster->lru, sib) {
		if (!num_cpus--)
			break;

		if (!is_active(c))
			continue;
		if (cluster->active_cpus == need)
			break;
		/* Don't isolate busy CPUs. */
		if (c->is_busy)
			continue;

		/*
		 * We isolate only the not_preferred CPUs. If none
		 * of the CPUs are selected as not_preferred, then
		 * all CPUs are eligible for isolation.
		 */
		if (cluster->nr_not_preferred_cpus && !c->not_preferred)
			continue;

		if (!should_we_isolate(c->cpu, cluster))
			continue;

		spin_unlock_irqrestore(&state_lock, flags);

		pr_debug("Trying to isolate CPU%u\n", c->cpu);
		if (!sched_isolate_cpu(c->cpu)) {
			c->isolated_by_us = true;
			move_cpu_lru(c);
			nr_isolated++;
		} else {
			pr_debug("Unable to isolate CPU%u\n", c->cpu);
		}
		cluster->active_cpus = get_active_cpu_count(cluster);
		spin_lock_irqsave(&state_lock, flags);
	}
	cluster->nr_isolated_cpus += nr_isolated;
	spin_unlock_irqrestore(&state_lock, flags);

again:
	/*
	 * If the number of active CPUs is within the limits, then
	 * don't force isolation of any busy CPUs.
	 */
	if (cluster->active_cpus <= cluster->max_cpus)
		return;

	nr_isolated = 0;
	num_cpus = cluster->num_cpus;
	spin_lock_irqsave(&state_lock, flags);
	list_for_each_entry_safe(c, tmp, &cluster->lru, sib) {
		if (!num_cpus--)
			break;

		if (!is_active(c))
			continue;
		if (cluster->active_cpus <= cluster->max_cpus)
			break;

		if (first_pass && !c->not_preferred)
			continue;

		spin_unlock_irqrestore(&state_lock, flags);

		pr_debug("Trying to isolate CPU%u\n", c->cpu);
		if (!sched_isolate_cpu(c->cpu)) {
			c->isolated_by_us = true;
			move_cpu_lru(c);
			nr_isolated++;
		} else {
			pr_debug("Unable to isolate CPU%u\n", c->cpu);
		}
		cluster->active_cpus = get_active_cpu_count(cluster);
		spin_lock_irqsave(&state_lock, flags);
	}
	cluster->nr_isolated_cpus += nr_isolated;
	spin_unlock_irqrestore(&state_lock, flags);

	if (first_pass && cluster->active_cpus > cluster->max_cpus) {
		first_pass = false;
		goto again;
	}
}

static void __try_to_unisolate(struct cluster_data *cluster,
			       unsigned int need, bool force)
{
	struct cpu_data *c, *tmp;
	unsigned long flags;
	unsigned int num_cpus = cluster->num_cpus;
	unsigned int nr_unisolated = 0;

	/*
	 * Protect against entry being removed (and added at tail) by other
	 * thread (hotplug).
	 */
	spin_lock_irqsave(&state_lock, flags);
	list_for_each_entry_safe(c, tmp, &cluster->lru, sib) {
		if (!num_cpus--)
			break;

		if (!c->isolated_by_us)
			continue;
		if ((cpu_online(c->cpu) && !cpu_isolated(c->cpu)) ||
			(!force && c->not_preferred))
			continue;
		if (cluster->active_cpus == need)
			break;

		spin_unlock_irqrestore(&state_lock, flags);

		pr_debug("Trying to unisolate CPU%u\n", c->cpu);
		if (!sched_unisolate_cpu(c->cpu)) {
			c->isolated_by_us = false;
			move_cpu_lru(c);
			nr_unisolated++;
		} else {
			pr_debug("Unable to unisolate CPU%u\n", c->cpu);
		}
		cluster->active_cpus = get_active_cpu_count(cluster);
		spin_lock_irqsave(&state_lock, flags);
	}
	cluster->nr_isolated_cpus -= nr_unisolated;
	spin_unlock_irqrestore(&state_lock, flags);
}

static void try_to_unisolate(struct cluster_data *cluster, unsigned int need)
{
	bool force_use_non_preferred = false;

	__try_to_unisolate(cluster, need, force_use_non_preferred);

	if (cluster->active_cpus == need)
		return;

	force_use_non_preferred = true;
	__try_to_unisolate(cluster, need, force_use_non_preferred);
}

static void __ref do_core_ctl(struct cluster_data *cluster)
{
	unsigned int need;

	need = apply_limits(cluster, cluster->need_cpus);

	if (adjustment_possible(cluster, need)) {
		pr_debug("Trying to adjust group %u from %u to %u\n",
				cluster->first_cpu, cluster->active_cpus, need);

		if (cluster->active_cpus > need)
			try_to_isolate(cluster, need);
		else if (cluster->active_cpus < need)
			try_to_unisolate(cluster, need);
	}
}

static int __ref try_core_ctl(void *data)
{
	struct cluster_data *cluster = data;
	unsigned long flags;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&cluster->pending_lock, flags);
		if (!cluster->pending) {
			spin_unlock_irqrestore(&cluster->pending_lock, flags);
			schedule();
			if (kthread_should_stop())
				break;
			spin_lock_irqsave(&cluster->pending_lock, flags);
		}
		set_current_state(TASK_RUNNING);
		cluster->pending = false;
		spin_unlock_irqrestore(&cluster->pending_lock, flags);

		do_core_ctl(cluster);
	}

	return 0;
}
EXPORT_SYMBOL(core_ctl_set_boost);

static int __init core_ctl_init(void)
{
	return 0;
}

late_initcall(core_ctl_init);

#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/tick.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/oem/houston.h>
#include <linux/perf_event.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/clk.h>

#include "../drivers/gpu/msm/kgsl.h"
#include "../drivers/gpu/msm/kgsl_pwrctrl.h"

#include <oneplus/houston/houston_helper.h>

#include <linux/smp.h>

#ifdef CONFIG_CONTROL_CENTER
#include <linux/oem/control_center.h>
#endif

/* perf raw counter */
#define ARMV8_PMCR_MASK 0x3f
#define ARMV8_PMCR_E (1 << 0) /* Enable all counters */
#define ARMV8_PMCR_C (1 << 2) /* Cycle counter reset */
#define ARMV8_PMCR_LC (1 << 6) /* Cycle Counter 64bit overflow */

/* Need to align housotn.h HT_MONITOR_SIZE */
static const char *ht_monitor_case[HT_MONITOR_SIZE] = {
	"ts",
	"clus_0_min", "clus_0_cur", "clus_0_max", "clus_0_iso",
	"clus_1_min", "clus_1_cur", "clus_1_max", "clus_1_iso",
	"clus_2_min", "clus_2_cur", "clus_2_max", "clus_2_iso",
	"gpu_cur", "voltage_now", "current_now", "hw_instruction",
	"hw_cache_miss", "hw_cycle",
	"cpu-0-0-usr", "cpu-0-1-usr", "cpu-0-2-usr", "cpu-0-3-usr",
	"cpu-1-0-usr", "cpu-1-1-usr", "cpu-1-2-usr", "cpu-1-3-usr",
	"cpu-1-4-usr", "cpu-1-5-usr", "cpu-1-6-usr", "cpu-1-7-usr",
	"skin-therm", "skin-msm-therm",
	"util-0", "util-1", "util-2", "util-3", "util-4",
	"util-5", "util-6", "util-7",
	"process name", "layer name", "pid", "fps_align", "actualFps",
	"predictFps", "appSwapTime", "appSwapDuration",
	"appEnqueueDuration", "sfTotalDuration", "sfPresentTime",
	"Vsync", "missedLayer", "render_pid", "render_util",
	"nt_rtg", "rtg_util_sum"
};

/*
 * log output
 * lv == 0 -> verbose info warning error
 * lv == 1 -> info warning error
 * lv == 2 -> wraning error
 * lv >= 3 -> error
 */
static int ht_log_lv = 1;
module_param_named(log_lv, ht_log_lv, int, 0664);

/* ais */
static int ais_enable = 0;
module_param_named(ais_enable, ais_enable, int, 0664);

/* pmu */
static int perf_ready = -1;

static struct workqueue_struct *ht_perf_workq;

static DECLARE_WAIT_QUEUE_HEAD(ht_perf_waitq);
static DECLARE_WAIT_QUEUE_HEAD(ht_poll_waitq);

/* render & rtg util*/
static pid_t RenPid = -1;
/*
 * perf event list
 * A list chained with task which perf event created
 */
static struct list_head ht_perf_event_head = LIST_HEAD_INIT(ht_perf_event_head);

static struct list_head ht_rtg_head = LIST_HEAD_INIT(ht_rtg_head);

/*
 * tmp list for storing rtg tasks
 * when traverse rtg list, we need hold spin lock, but in this context
 * we can't collect perf data (might sleep), so we need to use another
 * list to store these tasks, and then collect perf data in safe context.
 */
static struct list_head ht_rtg_perf_head = LIST_HEAD_INIT(ht_rtg_perf_head);


/*
 * filter mechanism
 * base_util: rtg task util threshold
 * rtg_filter_cnt: rtg task called cnt threshold under 1 sec
 */
static unsigned int base_util = 100;
module_param_named(base_util, base_util, uint, 0664);
static unsigned int rtg_filter_cnt = 10;
module_param_named(rtg_filter_cnt, rtg_filter_cnt, uint, 0664);

/* sched */
extern unsigned long long task_sched_runtime(struct task_struct *p);

/* cpuload tracking */
/* TODO these info maybe useless to sufraceflinger, should remove later */
struct cpuload_info {
	int cnt;
	int cmin;
	int cmax;
	int sum;
	long long iowait_min;
	long long iowait_max;
	long long iowait_sum;
};
static long long ht_iowait[8] = {0};
static long long ht_delta_iowait[8] = {0};
static bool cpuload_query = false;
module_param_named(cpuload_query, cpuload_query, bool, 0664);

/* battery query, it takes time to query */
static bool bat_query = false;
module_param_named(bat_query, bat_query, bool, 0664);

static bool bat_sample_high_resolution = false;
module_param_named(bat_sample_high_resolution, bat_sample_high_resolution, bool, 0664);

/* force update battery current */
static unsigned long bat_update_period_us = 1000000; // 1 sec
module_param_named(bat_update_period_us, bat_update_period_us, ulong, 0664);

extern void bq27541_force_update_current(void);

/* fps boost switch */
static bool fps_boost_enable = false;
module_param_named(fps_boost_enable, fps_boost_enable, bool, 0664);

/* freq hispeed */
static bool cpufreq_hispeed_enable = false;
module_param_named(cpufreq_hispeed_enable, cpufreq_hispeed_enable, bool, 0664);

static unsigned int cpufreq_hispeed[HT_CLUSTERS] = { 1209600, 1612800, 1612800 };
module_param_array_named(cpufreq_hispeed, cpufreq_hispeed, uint, NULL, 0664);

static bool ddrfreq_hispeed_enable = false;
module_param_named(ddrfreq_hispeed_enable, ddrfreq_hispeed_enable, bool, 0664);

static unsigned int ddrfreq_hispeed = 1017;
module_param_named(ddrfreq_hispeed, ddrfreq_hispeed, uint, 0664);

/* choose boost freq to lock or lower bound */
static unsigned int fps_boost_type = 1;
module_param_named(fps_boost_type, fps_boost_type, uint, 0664);

/* filter out too close boost hint */
static unsigned long fps_boost_filter_us = 8000;
module_param_named(fps_boost_filter_us, fps_boost_filter_us, ulong, 0664);

/* monitor switch */
static unsigned int ht_enable = 0;

/* houston monitor
 * data: sample data
 * layer: sample data for frame info
 * process: sample data for frame process info
 */
struct sample_data {
	u64 data[MAX_REPORT_PERIOD][HT_MONITOR_SIZE];
	char layer[MAX_REPORT_PERIOD][FPS_LAYER_LEN];
	char process[MAX_REPORT_PERIOD][FPS_PROCESS_NAME_LEN];
};

struct ht_monitor {
	struct power_supply *psy;
	struct thermal_zone_device* tzd[HT_MONITOR_SIZE];
	struct task_struct *thread;
	struct sample_data *buf;
} monitor = {
	.psy = NULL,
	.thread = NULL,
	.buf = NULL,
};

struct ht_util_pol {
	unsigned long *utils[HT_CPUS_PER_CLUS];
	unsigned long *hi_util;
};

/* mask only allow within 64 events */
static unsigned long ht_all_mask = 0;

static unsigned long filter_mask = 0;
module_param_named(filter_mask, filter_mask, ulong, 0664);

static unsigned long disable_mask = 0;
module_param_named(disable_mask, disable_mask, ulong, 0664);

static unsigned int report_div[HT_MONITOR_SIZE];
module_param_array_named(div, report_div, uint, NULL, 0664);

static unsigned int sample_rate = 3000;

/*
 * monitor configuration
 * sidx: current used idx (should be update only by monitor thread)
 * record_cnt: current recorded sample amount
 * cached_fps: to record current efps and fps info
 * cached_layer_name: to record layer name. (debug purpose)
 * ht_tzd_idx: thermal zone index
 * gpwe: saved kgsl ptr, to get gpu freq
 * sample_rate: sample rate in ms
 * ht_utils: saved util prt, update from sugov
 * keep_alive: monitor life cycle
 */
static int sidx;

static dev_t ht_ctl_dev;
static struct class *driver_class;
static struct cdev cdev;

/* helper */
static inline int cpu_to_clus(int cpu)
{
	switch (cpu) {
	case 0: case 1: case 2: case 3: return 0;
	case 4: case 5: case 6: return 1;
	case 7: return 2;
	}
	return 0;
}

static inline int clus_to_cpu(int clus)
{
	switch (clus) {
	case 0: return CLUS_0_IDX;
	case 1: return CLUS_1_IDX;
	case 2: return CLUS_2_IDX;
	}
	return CLUS_0_IDX;
}

static inline u64 ddr_find_target(u64 target) {
	int i;
	u64 ddr_options[11] = {
		200, 300, 451, 547, 681, 768, 1017, 1353, 1555, 1804, 2092
	};

	for (i = 10; i >= 0; --i) {
		if (target >= ddr_options[i]) {
			target = ddr_options[i];
			break;
		}
	}
	return target;
}

static inline void ht_query_ddrfreq(u64* val)
{
	clk_get_ddr_freq(val);

	*val /= 1000000;
	/* process for easy deal with */
	if (*val == 1018) *val = 1017;
	else if (*val == 1355) *val = 1353;
	else if (*val == 1805) *val = 1804;
	else if (*val == 2096) *val = 2092;
}

static inline int ht_next_sample_idx(void)
{
	++sidx;
	sidx %= MAX_REPORT_PERIOD;

	return sidx;
}

static inline void ht_set_all_mask(void)
{
	int i;

	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		ht_all_mask |= (1L << i);
}

static inline bool ht_is_all_disabled(unsigned long mask)
{
	return ht_all_mask == mask;
}

static inline bool ht_is_all_filtered(unsigned long mask)
{
	return ht_all_mask == mask;
}

static inline int ht_mapping_tags(char *name)
{
	int i;

	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		if (!strcmp(name, ht_monitor_case[i]))
			return i;

	return HT_MONITOR_SIZE;
}

static inline const char* ht_ioctl_str(unsigned int cmd)
{
	switch (cmd) {
	case HT_IOC_COLLECT: return "HT_IOC_COLLECT";
	case HT_IOC_SCHEDSTAT: return "HT_IOC_SCHEDSTAT";
	case HT_IOC_CPU_LOAD: return "HT_IOC_CPU_LOAD";
	}
	return "NONE";
}

static inline int ht_get_temp(int monitor_idx)
{
	return 0;
}

static inline void ht_update_battery(void)
{
	return;
}

static inline u64 ht_get_iowait_time(int cpu)
{
	u64 iowait, iowait_usecs = -1ULL;

	if (cpu_online(cpu))
		iowait_usecs = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
		iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	else
		iowait = iowait_usecs * NSEC_PER_USEC;

	return iowait;
}

/* offline included */
static inline int ht_iso_count(const cpumask_t *mask)
{
	cpumask_t count_mask = CPU_MASK_NONE;

	cpumask_complement(&count_mask, cpu_online_mask);
	cpumask_or(&count_mask, &count_mask, cpu_isolated_mask);
	cpumask_and(&count_mask, &count_mask, mask);

	return cpumask_weight(&count_mask);
}

/* sched switch update */
static inline void ht_sched_update(struct task_struct *task, bool in)
{
	return;
}

static inline u32 armv8pmu_pmcr_read(void)
{
	u64 val = 0;
	asm volatile("mrs %0, pmcr_el0" : "=r" (val));
	return (u32)val;
}

static inline void armv8pmu_pmcr_write(u32 val)
{
	val &= ARMV8_PMCR_MASK;
	isb();
	asm volatile("msr pmcr_el0, %0" : : "r" ((u64)val));
}

void ht_rtg_init(struct task_struct *task)
{
	return;
}

static int perf_ready_store(const char *buf, const struct kernel_param *kp)
{
	return 0;
}

static int perf_ready_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", perf_ready);
}

static struct kernel_param_ops perf_ready_ops = {
	.set = perf_ready_store,
	.get = perf_ready_show,
};
module_param_cb(perf_ready, &perf_ready_ops, NULL, 0664);

static inline void __ht_perf_event_enable(struct task_struct *task, int event_id)
{
	return;
}

static inline void ht_perf_event_enable(struct task_struct *task)
{
	return;
}

static inline void __ht_perf_event_read(struct task_struct *task, struct ai_thread_parcel* t, int event_id)
{
	return;
}

static inline void ht_perf_event_read(struct task_struct *task, struct ai_thread_parcel* t)
{
	return;
}

static inline void ht_collect_parcel_data(
	struct task_struct *task,
	struct ai_parcel* parcel,
	int pidx,
	bool is_enqueue_task)
{
	return;
}

void ht_collect_perf_data(struct work_struct *work)
{
	return;
}

static int ht_enable_store(const char *buf, const struct kernel_param *kp)
{
	return 0;
}

static int ht_enable_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", ht_enable);
}

static struct kernel_param_ops ht_enable_ops = {
	.set = ht_enable_store,
	.get = ht_enable_show,
};
module_param_cb(ht_enable, &ht_enable_ops, NULL, 0664);

static int sample_rate_store(const char *buf, const struct kernel_param *kp)
{
	return 0;
}

static int sample_rate_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", sample_rate);
}

static struct kernel_param_ops sample_rate_ops = {
	.set = sample_rate_store,
	.get = sample_rate_show,
};
module_param_cb(sample_rate_ms, &sample_rate_ops, NULL, 0664);

static int ht_fps_boost_store(const char *buf, const struct kernel_param *kp)
{
	return 0;
}

static struct kernel_param_ops ht_fps_boost_ops = {
	.set = ht_fps_boost_store,
};
module_param_cb(fps_boost, &ht_fps_boost_ops, NULL, 0220);

void ht_register_kgsl_pwrctrl(void *pwr)
{
	return;
}

void ht_register_cpu_util(unsigned int cpu, unsigned int first_cpu,
		unsigned long *util, unsigned long *hi_util)
{
	return;
}

/* monitor hw event */
void ht_update_hw_events(u64 inst, u64 miss, u64 cycle)
{
	return;
}

void ht_register_thermal_zone_device(struct thermal_zone_device *tzd)
{
	return;
}

void ht_register_power_supply(struct power_supply *psy)
{
	return;
}

static int ht_fps_data_sync_store(const char *buf, const struct kernel_param *kp)
{
	return 0;
}

static struct kernel_param_ops ht_fps_data_sync_ops = {
	.set = ht_fps_data_sync_store,
};
module_param_cb(fps_data_sync, &ht_fps_data_sync_ops, NULL, 0220);

/* poll for fps data synchronization */
static unsigned int ht_ctl_poll(struct file *fp, poll_table *wait)
{
	poll_wait(fp, &ht_poll_waitq, wait);
	return POLLIN;
}

static inline void ht_cpuload_helper(int clus, int cpus, struct cpuload_info *cli)
{
	return;
}

static long ht_ctl_ioctl(struct file *file, unsigned int cmd, unsigned long __user arg)
{
	if (_IOC_TYPE(cmd) != HT_IOC_MAGIC) return 0;
	if (_IOC_NR(cmd) > HT_IOC_MAX) return 0;

	switch (cmd) {
	case HT_IOC_COLLECT:
	{
		DEFINE_WAIT(wait);
		prepare_to_wait(&ht_perf_waitq, &wait, TASK_INTERRUPTIBLE);
		schedule();
		finish_wait(&ht_perf_waitq, &wait);
		
		return 0;
		break;
	}
	case HT_IOC_SCHEDSTAT:
	{
		struct task_struct *task;
		u64 exec_ns = 0;
		u64 pid;
		if (copy_from_user(&pid, (u64 *) arg, sizeof(u64)))
			return 0;

		rcu_read_lock();
		task = find_task_by_vpid(pid);
		if (task) {
			get_task_struct(task);
			rcu_read_unlock();
			exec_ns = task_sched_runtime(task);
			put_task_struct(task);
		} else {
			rcu_read_unlock();
		}
		if (copy_to_user((u64 *) arg, &exec_ns, sizeof(u64)))
			return 0;
		break;
	}
	case HT_IOC_CPU_LOAD:
	{
		struct task_struct *task;
		struct cpuload_info cli = {
			0, INT_MAX, INT_MIN, 0, LONG_MAX, LONG_MIN, 0
		};
		struct cpuload cl;
		int clus = 0;
		int cpu;
		int i;

		if (!cpuload_query) {
			return 0;
		}

		if (copy_from_user(&cl, (struct cpuload *) arg, sizeof(struct cpuload)))
			return 0;

		rcu_read_lock();
		task = find_task_by_vpid(cl.pid);
		if (task)
			clus |= 1 << cpu_to_clus(task->cpu);
		task = find_task_by_vpid(cl.tid);
		if (task)
			clus |= 1 << cpu_to_clus(task->cpu);
		rcu_read_unlock();

		/* update iowait info */
		for_each_online_cpu(cpu) {
			long long iowait = ht_get_iowait_time(cpu);
			ht_delta_iowait[cpu] = iowait - ht_iowait[cpu];
			ht_iowait[cpu] = iowait;
		}

		for (i = 0; i < 3; ++i) {
			if (clus & (1 << i)) {
				switch (i) {
					case 0:
						ht_cpuload_helper(0, 4, &cli);
						break;
					case 1:
						ht_cpuload_helper(1, 3, &cli);
						break;
					case 2:
						ht_cpuload_helper(2, 1, &cli);
						break;
				}
			}
		}
		cl.min = cli.cmin;
		cl.max = cli.cmax;
		cl.sum = cli.sum;
		cl.avg = cli.cnt? cli.sum/cli.cnt: 0;
		cl.iowait_min = cli.iowait_min;
		cl.iowait_max = cli.iowait_max;
		cl.iowait_sum = cli.iowait_sum;
		cl.iowait_avg = cli.cnt? cli.iowait_sum/cli.cnt: 0;

		if (copy_to_user((struct cpuload *) arg, &cl, sizeof(struct cpuload)))
			return 0;
		break;
	}
	}
	return 0;
}

static const struct file_operations ht_ctl_fops = {
	.owner = THIS_MODULE,
	.poll = ht_ctl_poll,
	.unlocked_ioctl = ht_ctl_ioctl,
	.compat_ioctl = ht_ctl_ioctl,
};

static int ht_registered_show(char* buf, const struct kernel_param *kp)
{
	return 0;
}

static struct kernel_param_ops ht_registed_ops = {
	.get = ht_registered_show,
};
module_param_cb(ht_registed, &ht_registed_ops, NULL, 0444);

static int ht_reset_store(const char *buf, const struct kernel_param *kp)
{
	return 0;
}

static struct kernel_param_ops ht_reset_ops = {
	.set = ht_reset_store,
};
module_param_cb(reset, &ht_reset_ops, NULL, 0664);

static int ht_report_proc_show(struct seq_file *m, void *v)
{
	return 0;
}

static int ht_report_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ht_report_proc_show, NULL);
}

static const struct file_operations ht_report_proc_fops = {
	.open= ht_report_proc_open,
	.read= seq_read,
	.llseek= seq_lseek,
	.release= single_release,
};

static int fps_sync_init(void)
{
	int rc;
	struct device *class_dev;

	rc = alloc_chrdev_region(&ht_ctl_dev, 0, 1, HT_CTL_NODE);
	if (rc < 0) {
		ht_loge("alloc_chrdev_region failed %d\n", rc);
		return 0;
	}

	driver_class = class_create(THIS_MODULE, HT_CTL_NODE);
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		ht_loge("class_create failed %d\n", rc);
		goto exit_unreg_chrdev_region;
	}
	class_dev = device_create(driver_class, NULL, ht_ctl_dev, NULL, HT_CTL_NODE);
	if (IS_ERR(class_dev)) {
		ht_loge("class_device_create failed %d\n", rc);
		rc = -ENOMEM;
		goto exit_destroy_class;
	}
	cdev_init(&cdev, &ht_ctl_fops);
	cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev, MKDEV(MAJOR(ht_ctl_dev), 0), 1);
	if (rc < 0) {
		ht_loge("cdev_add failed %d\n", rc);
		goto exit_destroy_device;
	}
	ht_logi("fps data sync ready\n");
	return 0;
exit_destroy_device:
	device_destroy(driver_class, ht_ctl_dev);
exit_destroy_class:
	class_destroy(driver_class);
exit_unreg_chrdev_region:
	unregister_chrdev_region(ht_ctl_dev, 1);
	return 0;
}

void ht_update_enqueue_ts(struct task_struct *task)
{
	return;
}

void ht_perf_notify(void)
{
	u64 time;

	if (perf_ready <= 0)
		return;

	if (perf_ready != current->tgid) {
		current->enqueue_ts = 0;
		ht_logv("notify: task %s %d ignored due to not target process\n", current->comm, current->pid);
		return;
	}

	/* if thread been tracked by perf, skip it */
	if (current->perf_regular_activate) {
		ht_logv("notify: task %s %d ignored due to perf already activated\n", current->comm, current->pid);
		return;
	}

	ht_logv("notify: task: comm: %s, pid: %d, tgid: %d, freq: %u, peak: %u, leader: %s %d %d, ts: %lld\n",
		current->comm, current->pid, current->tgid, current->f_cnt, current->f_peak,
		current->group_leader->comm, current->group_leader->pid, current->group_leader->tgid,
		current->enqueue_ts);

	RenPid = current->pid;

	/* fps info update */
	wake_up_interruptible(&ht_poll_waitq);

	/* get timestamp */
	time = ktime_to_us(ktime_get());
	ht_update_enqueue_ts(current);
	current->enqueue_ts = time;

	/* may need to handle overflow */
	if (time < current->f_ts ||
		time - current->f_ts >= 1000000 /* 1 sec */) {
		current->f_peak = current->f_cnt;
		current->f_cnt = 0;
		current->f_ts = time;
	}
	++current->f_cnt;

	if (likely(ht_perf_workq)) {
		queue_work(ht_perf_workq, &current->perf_work);
	}
}

void ht_perf_event_init(struct task_struct *task)
{
	int i;

	INIT_LIST_HEAD(&task->ht_perf_event_node);
	INIT_LIST_HEAD(&task->perf_node);
	INIT_WORK(&task->perf_work, ht_collect_perf_data);
	for (i = 0; i < HT_PERF_COUNT_MAX; ++i) {
		task->perf_events[i] = NULL;
		task->perf_counters[i] = 0;
	}
	task->perf_activate = 0;
	task->perf_regular_activate = 0;
	task->enqueue_ts = 0;
	task->run_ts = 0;
	task->end_ts = 0;
	task->acc_run_ts = 0;
	task->delta_ts = 0;
	task->total_run_ts = 0;
}

void ht_perf_event_release(struct task_struct *task)
{
	return;
}

void ht_sched_switch_update(struct task_struct *prev, struct task_struct *next)
{
	return;
}

/* perform LRU order */
void ht_rtg_list_add_tail(struct task_struct *task)
{
	return;
}
EXPORT_SYMBOL(ht_rtg_list_add_tail);

void ht_rtg_list_del(struct task_struct *task)
{
	return;
}
EXPORT_SYMBOL(ht_rtg_list_del);

static int rtg_dump_show(char *buf, const struct kernel_param *kp)
{
	return 0;
}

static struct kernel_param_ops rtg_dump_ops = {
	.get = rtg_dump_show,
};
module_param_cb(rtg_dump, &rtg_dump_ops, NULL, 0444);

static int ht_init(void)
{
	fps_sync_init();
	return 0;
}
pure_initcall(ht_init);

/*
 * Copyright (c) 2018-2019 HUAWEI Technologies Co., Ltd.
 *
 * This source code is released for free distribution under the terms of the
 * GNU General Public License
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author:       os kernel lab
 * Created Time: Apr 20, 2019 11:12:58 AM CST
 * File Name:    file_map.c
 *
 * Description:
 */

#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/blk-cgroup.h>
#include <linux/file_map.h>
#include <linux/dcache.h>
#include <linux/fs_struct.h>
#include <asm/current.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/genhd.h>
#include <linux/atomic.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <asm-generic/checksum.h>
#include <linux/timer.h>

/******************************************************/
/*                     common                         */
/******************************************************/
enum fm_Log_level {
	log_level_error = 0,
	log_level_warning,
	log_level_info,
	log_level_debug,
	log_level_max
};

static enum fm_Log_level fm_print_level = log_level_info;

#define fm_print(level, fmt, ...) do {\
	if (level <= fm_print_level) { \
		pr_info("[fm][%s]"fmt, __func__, ##__VA_ARGS__); \
	} \
} while (0)

#define FILE_MAP_BLKSIZE_BITS       (12)
#define FILE_MAP_BLKSIZE            (4096)
#define FILE_MAP_BLK_ALIGN(x) \
	(((x) + FILE_MAP_BLKSIZE - 1) >> FILE_MAP_BLKSIZE_BITS)

/*
 * used by the service, modifying the value requires synchronization service
 * modification
 */
/* apk's Linux uid is from 10000 to larger */
#define APK_UID_MIN  ((uid_t)10000)
#define APK_UID_MAX  ((uid_t)65535)

enum file_map_state {
	FILE_MAP_STATE_STUDY = 0,
	FILE_MAP_STATE_USING,
	FILE_MAP_STATE_DISABLE,
	FILE_MAP_STATE_MAX,
};

enum file_map_result {
	FILE_MAP_RES_STUDY_COMPLETE = 0,
	FILE_MAP_RES_STUDY_NO_FILE,
	FILE_MAP_RES_USING_BITMAP_INVALID,
	FILE_MAP_RES_USING_BITMAP_LOWHIT,
	FILE_MAP_RES_USING_BITMAP_OK,
	FILE_MAP_RES_USING_BITMAP_OUTDATE,
	FILE_MAP_RES_FUN_NOT_ENABLE,
	FILE_MAP_RES_MAX,
};

static long fm_frag_threshold = 4;

/******************************************************/
/*                     common                         */
/******************************************************/
enum file_map_fs_type {
	FS_TYPE_F2FS = 0,
	FS_TYPE_EROFS,
	FS_TYPE_EXT4,
	FS_TYPE_MAX,
};

/******************************************************/
/*                ... device driver                   */
/******************************************************/
/*
 * device driver command and parameter definition used by the service,
 * modifying the value requires synchronization service modification.
 */
struct app_cold_launch_begin_para {
	uid_t uid;
	uint32_t map_size;
	enum file_map_state state;
};

#define FILE_MAP_IOC_MAGIC		'9'
#define FILE_MAP_COLD_LAUNCH_BEGIN	\
	_IOW(FILE_MAP_IOC_MAGIC, 1, struct app_cold_launch_begin_para)
#define FILE_MAP_COLD_LAUNCH_END	_IOW(FILE_MAP_IOC_MAGIC, 2, __u32)
#define FILE_MAP_ENABLE_CTL		_IOW(FILE_MAP_IOC_MAGIC, 6, int)
#define FILE_MAP_GET_RESULT		_IOR(FILE_MAP_IOC_MAGIC, 7, __u32)

#define FILE_MAP_IOC_MAXNR		10

static bool file_map_bitmap_enable; /* function global control */

/******************************************************/
/*           ... file map file format                 */
/******************************************************/
/* the file using bitmap must bigger than  */
#define FILE_MAP_FILE_SIZE_MIN		(3UL * 1024 * 1024)
/* the file using bitmap must smaller than  */
#define FILE_MAP_FILE_SIZE_MAX		(1UL * 1024 * 1024 * 1024)
/* 4k mapping 128M, max support 1G */
#define FILE_MAP_FILE_SINGLE_SIZE_MAX	(32UL * 1024)
#define FILE_MAP_MAX_FILES_PER_UID	(48)
#define FILE_MAP_MAPHEAD_MAGIC		(0x55aaaa55)
#define FILE_MAP_FILE_MAGIC		(0xaabbccdd)
#define FILE_MAP_FILE_HEAD_LEN		(sizeof(struct bitmap_file_hdr))
#define FILE_MAP_HEAD_LEN		(sizeof(struct bitmap_hdr))

#define FILE_MAP_FILE_MIN_SIZE		FILE_MAP_FILE_HEAD_LEN

/*
 * used by the service, modifying the value requires synchronization
 * service modification
 */
#define FILE_MAP_FILE_MAX_SIZE  (FILE_MAP_FILE_HEAD_LEN + \
	(FILE_MAP_MAX_FILES_PER_UID * \
	(FILE_MAP_HEAD_LEN + FILE_MAP_FILE_SINGLE_SIZE_MAX)))

struct bitmap_file_hdr {
	__le32 crc;	/* crc value, MUST be placed at the beginning */
	__le32 magic;
	__le32 uid;
	__le32 nr_bitmaps;	/* number of files */
	/* Starting offset of each bitmap file */
	__le32 bitmaps_header_offset[FILE_MAP_MAX_FILES_PER_UID];
	__le16 version;
	__le16 rsv16;
	__le32 rsv[23];
} __packed;

struct bitmap_hdr {
	__le32 magic;
	__le32 fs_type;		/* filesystem types */
	__le32 ino;		/* inode number */
	__le32 start_file_offset;	/* start offset of this bitmap file */
	__le32 end_file_offset;		/* end offset of this bitmap file */
	__le32 bitmap_offset;	/* bitmap data offset of this bitmap file */
	__le32 size;		/* bitmap data size in bytes */
	__le64 mtime;		/* modification time in second */
	__le32 mtime_nsec;	/* modification time in nanosecond */
	__le32 generation;	/* file version  */
	__le32 kdev;
	__le32 rsv[23];
} __packed;

struct file_map_entry {
	struct list_head list;
	uid_t uid;
	struct inode *inode; /* study only used: list inode maps/size used */
	unsigned long ino;
	enum file_map_fs_type fs_type;
	struct timespec mtime;
	uint32_t generation;
	dev_t kdev;
	uint32_t size;
	unsigned long maps[0]; /* keep in last */
};

struct file_map_s {
	uid_t uid;
	enum file_map_state state;
	enum file_map_result res;
	spinlock_t s_lock;
	uint32_t nr_bitmaps;
	struct list_head used;
	struct list_head unused;
	struct timer_list timer;
};

static struct file_map_s file_map = {
	.uid = -1,
	.state = FILE_MAP_STATE_DISABLE,
	.res = FILE_MAP_RES_MAX,
	.s_lock = __SPIN_LOCK_UNLOCKED(file_map.s_lock),
	.used = LIST_HEAD_INIT(file_map.used),
	.unused = LIST_HEAD_INIT(file_map.unused)
};

/******************************************************/
/*                    hitrate stat                    */
/******************************************************/

struct file_hitrate_s {
	unsigned long ino;
	atomic64_t miss;
	atomic64_t hit;
};

struct file_map_uid_hitrate_s {
	atomic64_t miss;
	atomic64_t hit;
	struct file_hitrate_s file_hitrate[FILE_MAP_MAX_FILES_PER_UID];
};

static long fm_low_hitrate_threshold = 75;
static struct file_map_uid_hitrate_s hitrate_stat;

/******************************************************/
/*           readahead bitmap usage stat              */
/******************************************************/
struct file_map_stat_s {
	atomic64_t total_pages;
	atomic64_t ignore_pages;
	atomic64_t shrink_bg_pages;
	atomic64_t shrink_fg_pages;
	atomic64_t expand_fg_pages;
	atomic64_t lowhit_times;
	atomic64_t highhit_times;
	atomic64_t readaround_times;
	atomic64_t readforward_times;
};

struct file_map_stat_s file_map_stat;

#ifdef FILE_MAP_DEBUG
/******************************************************/
/*                    history record                  */
/******************************************************/
#define FILE_MAP_MAX_HISTORY_RECORD_NUM             (512)

struct file_map_history_item_s {
	struct list_head list;
	uid_t uid;
	enum file_map_state state;
	enum file_map_result res;
	long hitrate;
};

struct file_map_history_list_s {
	struct list_head head;
	uint32_t nr_items; /* max FILE_MAP_MAX_HISTORY_RECORD_NUM */
	spinlock_t lock;
};

static struct file_map_history_list_s file_map_history_list  = {
	.head = LIST_HEAD_INIT(file_map_history_list.head),
	.lock = __SPIN_LOCK_UNLOCKED(file_map_history_list.lock)
};
#endif

static uint32_t get_inode_bitmap_size(struct inode *inode)
{
	return BITS_TO_LONGS(
		FILE_MAP_BLK_ALIGN((unsigned long)i_size_read(inode))) *
		sizeof(unsigned long);
}

/*
 * Function name: file_map_stat_total_inc
 * Discription: increase readahead total pages
 * Parameters:
 *      @ long cnt
 * return value:
 *      @ NULL
 */
void file_map_stat_total_inc(long cnt)
{
	atomic64_add(cnt, &file_map_stat.total_pages);
}
EXPORT_SYMBOL(file_map_stat_total_inc);

/*
 * Function name: file_map_stat_ignore_inc
 * Discription: increase readahead ignore pages
 * Parameters:
 *      @ long cnt
 * return value:
 *      @ NULL
 */
void file_map_stat_ignore_inc(long cnt)
{
	atomic64_add(cnt, &file_map_stat.ignore_pages);
}
EXPORT_SYMBOL(file_map_stat_ignore_inc);

static bool file_map_check_fs_name_valid(struct inode *inode)
{
	return (inode != NULL && inode->i_sb != NULL &&
		inode->i_sb->s_type != NULL &&
		inode->i_sb->s_type->name != NULL);
}

static bool file_map_check_fs_type(enum file_map_fs_type fs_type)
{
	return (fs_type == FS_TYPE_F2FS || fs_type == FS_TYPE_EROFS ||
		fs_type == FS_TYPE_EXT4);
}

static bool file_map_uid_valid(uid_t uid)
{
	return (uid >= APK_UID_MIN && uid <= APK_UID_MAX);
}

static bool file_map_check_cur_uid(uid_t cur_uid)
{
	uid_t uid = current_uid().val;

	if (uid == cur_uid)
		return true;

	return false;
}

static enum file_map_fs_type file_map_get_fs_type(struct inode *inode)
{
	const char *fs_name = NULL;

	if (!file_map_check_fs_name_valid(inode))
		return FS_TYPE_MAX;

	fs_name = inode->i_sb->s_type->name;
	if (strstr(fs_name, "f2fs"))
		return FS_TYPE_F2FS;
	else if (strstr(fs_name, "erofs"))
		return FS_TYPE_EROFS;
	else if (strstr(fs_name, "ext4"))
		return FS_TYPE_EXT4;
	else
		return FS_TYPE_MAX;
}

static bool check_extension(const unsigned char *s)
{
	size_t slen = strlen(s);
	int i;
	const unsigned char *ext = NULL;

	if (slen < 2 || s[slen - 1] == '.')
		return false;

	for (i = slen - 2; i >= 0; i--) {
		if (s[i] == '.') {
			ext = (s + i);
			break;
		}
	}

	if (!ext)
		return false;

	if (!strcmp(ext, ".apk") || !strcmp(ext, ".so") ||
	    !strcmp(ext, ".vdex") || !strcmp(ext, ".odex") ||
	    !strcmp(ext, ".jar"))
		return true;

	return false;
}

static bool file_map_is_foreground(void)
{
#ifdef CONFIG_BLK_CGROUP
	struct blkcg *blkcg = NULL;

	rcu_read_lock();
	blkcg = task_blkcg(current);
	if (blkcg && (blkcg->type == BLK_THROTL_FG ||
		blkcg->type == BLK_THROTL_TA ||
		blkcg->type == BLK_THROTL_KBG)) {
		rcu_read_unlock();
		return true;
	}
	rcu_read_unlock();
#endif
	return false;
}

static bool file_map_file_prefilter(struct file *filp,
				    const unsigned char *name,
				    enum file_map_fs_type *p_fs_type)
{
	struct inode *inode = filp->f_inode;
	loff_t size = i_size_read(inode);
	enum file_map_fs_type fs_type;

	if (!file_map_is_foreground())
		return false;

	fs_type = file_map_get_fs_type(inode);
	if (!file_map_check_fs_type(fs_type))
		return false;

	if (!S_ISREG(inode->i_mode) || size < FILE_MAP_FILE_SIZE_MIN ||
	    size > FILE_MAP_FILE_SIZE_MAX)
		return false;

	*p_fs_type = fs_type;

	return check_extension(name);
}

static void file_map_hitrate_clear(void)
{
	unsigned long i;

	atomic64_set(&hitrate_stat.hit, 0);
	atomic64_set(&hitrate_stat.miss, 0);

	for (i = 0; i < FILE_MAP_MAX_FILES_PER_UID; i++) {
		hitrate_stat.file_hitrate[i].ino = 0;
		atomic64_set(&hitrate_stat.file_hitrate[i].hit, 0);
		atomic64_set(&hitrate_stat.file_hitrate[i].miss, 0);
	}
}

static long file_map_get_hitrate(void)
{
	long h, m, t;

	h = atomic64_read(&hitrate_stat.hit);
	m = atomic64_read(&hitrate_stat.miss);

	t = h + m;
	if (t < 1)
		t = 1;

	return (h * 100) / t;
}

static void file_map_hitrate_update_miss(struct inode *inode)
{
	unsigned long i;

	for (i = 0; i < FILE_MAP_MAX_FILES_PER_UID; i++) {
		if (inode->i_ino == hitrate_stat.file_hitrate[i].ino) {
			atomic64_inc(&hitrate_stat.file_hitrate[i].miss);
			atomic64_inc(&hitrate_stat.miss);
			break;
		}
	}
}

static void file_map_hitrate_update_hit(struct inode *inode)
{
	unsigned long i;

	for (i = 0; i < FILE_MAP_MAX_FILES_PER_UID; i++) {
		if (inode->i_ino == hitrate_stat.file_hitrate[i].ino) {
			atomic64_inc(&hitrate_stat.file_hitrate[i].hit);
			atomic64_inc(&hitrate_stat.hit);
			break;
		}
	}
}

static void file_map_hitrate_set_ino(unsigned long idx, unsigned long ino)
{
	if (likely(idx < FILE_MAP_MAX_FILES_PER_UID))
		hitrate_stat.file_hitrate[idx].ino = ino;
}

#ifdef FILE_MAP_DEBUG
static void file_map_hitrate_show(void)
{
	unsigned long i = 0;
	long h, m, r;
	struct file_map_uid_hitrate_s hitrate_tmp, *ph = &hitrate_tmp;

	memcpy(ph, &hitrate_stat, sizeof(hitrate_stat));

	h = atomic_read(&ph->hit);
	m = atomic_read(&ph->miss);
	r = ((h + m) == 0) ? 0 : ((h * 100) / (h + m));

	fm_print(log_level_info, "[file map hitrate]\n");
	fm_print(log_level_info,
		"uid total:       hit: %-10ld miss: %-10ld hitrate: %ld%%\n",
		h, m, r);

	for (i = 0; i < FILE_MAP_MAX_FILES_PER_UID; i++) {
		if (ph->file_hitrate[i].ino != 0) {
			h = atomic_read(&ph->file_hitrate[i].hit);
			m = atomic_read(&ph->file_hitrate[i].miss);
			r = ((h + m) == 0) ? 0 : ((h * 100) / (h + m));
			fm_print(log_level_info,
				"ino: %-10lu  hit: %-10ld miss: %-10ld hitrate: %ld%%\n",
				ph->file_hitrate[i].ino, h, m, r);
		}
	}
}
#endif

static bool file_map_check_in_study(void)
{
	return (file_map.state == FILE_MAP_STATE_STUDY);
}

static bool file_map_check_in_using(void)
{
	return (file_map.state == FILE_MAP_STATE_USING);
}

static bool file_map_check_support(void)
{
	if (!file_map_bitmap_enable)
		return false;

	if (!file_map_is_foreground())
		return false;

	return true;
}

static unsigned long file_map_extract_item(struct bitmap_file_hdr *pfhdr,
					struct file_map_entry *pentry,
					char *buf,
					unsigned long file_save_pos,
					unsigned long save_cnt)
{
	unsigned long page_bitmap_size;
	unsigned long *page_bitmaps = NULL;
	struct bitmap_hdr *pbhdr = NULL;

	if (!pentry->size)
		return 0;

	page_bitmaps = pentry->maps;
	page_bitmap_size = pentry->size;

	if ((file_save_pos + page_bitmap_size +
		FILE_MAP_HEAD_LEN) >= FILE_MAP_FILE_MAX_SIZE) {
		fm_print(log_level_info, "study content too large.\n");
		return 0;
	}

	pbhdr = (struct bitmap_hdr *)(buf + file_save_pos);
	pbhdr->magic = cpu_to_le32(FILE_MAP_MAPHEAD_MAGIC);
	pbhdr->fs_type = cpu_to_le32(pentry->fs_type);
	pbhdr->ino = cpu_to_le32(pentry->ino);
	pbhdr->start_file_offset = cpu_to_le32(file_save_pos);
	pbhdr->end_file_offset = cpu_to_le32(pbhdr->start_file_offset +
					   page_bitmap_size +
					   FILE_MAP_HEAD_LEN);
	pbhdr->bitmap_offset = cpu_to_le32(file_save_pos +
					 FILE_MAP_HEAD_LEN);
	pbhdr->size = cpu_to_le32(page_bitmap_size);
	pbhdr->mtime = cpu_to_le64(pentry->mtime.tv_sec);
	pbhdr->mtime_nsec = cpu_to_le32(pentry->mtime.tv_nsec);
	pbhdr->generation = cpu_to_le32(pentry->generation);
	pbhdr->kdev = cpu_to_le32(pentry->kdev);

	pfhdr->bitmaps_header_offset[save_cnt] =
		cpu_to_le32(pbhdr->start_file_offset);

	memset(pbhdr->rsv, 0, sizeof(pbhdr->rsv));
	memcpy(buf + file_save_pos + FILE_MAP_HEAD_LEN,
		page_bitmaps, page_bitmap_size);

	return page_bitmap_size;
}

/* file_map.s_lock held */
static size_t file_map_study_content_extract(uid_t uid, char *buf)
{
	struct bitmap_file_hdr *pfhdr = (struct bitmap_file_hdr *)buf;
	struct file_map_entry *pentry = NULL;
	unsigned long file_save_pos = 0;
	unsigned long page_bitmap_size;
	unsigned long save_cnt;
	__wsum csum;

	save_cnt = 0;
	file_save_pos = FILE_MAP_FILE_HEAD_LEN;
	list_for_each_entry(pentry, &file_map.used, list) {
		page_bitmap_size = file_map_extract_item(pfhdr, pentry,
						buf, file_save_pos, save_cnt);
		if (page_bitmap_size) {
			save_cnt++;
			file_save_pos += page_bitmap_size + FILE_MAP_HEAD_LEN;
		}
	}
	list_for_each_entry(pentry, &file_map.unused, list) {
		page_bitmap_size = file_map_extract_item(pfhdr, pentry,
						buf, file_save_pos, save_cnt);
		if (page_bitmap_size) {
			save_cnt++;
			file_save_pos += page_bitmap_size + FILE_MAP_HEAD_LEN;
		}
	}

	if (save_cnt == 0)
		return 0;

	pfhdr->nr_bitmaps = cpu_to_le32(save_cnt);
	pfhdr->magic = cpu_to_le32(FILE_MAP_FILE_MAGIC);
	pfhdr->uid = cpu_to_le32(uid);
	pfhdr->version = 1;
	pfhdr->rsv16 = 0;
	memset(pfhdr->rsv, 0, sizeof(pfhdr->rsv));
	csum = csum_partial(buf + sizeof(__le32),
			    file_save_pos - sizeof(__le32), 0);
	pfhdr->crc = cpu_to_le32(csum);

	return file_save_pos;
}


#ifdef FILE_MAP_DEBUG
static void file_map_history_add(struct file_map_history_item_s *new)
{
	struct file_map_history_item_s *item = NULL;

	spin_lock(&file_map_history_list.lock);
	list_add_tail(&new->list, &file_map_history_list.head);
	if (file_map_history_list.nr_items < FILE_MAP_MAX_HISTORY_RECORD_NUM) {
		file_map_history_list.nr_items++;
	} else {
		item = list_entry(file_map_history_list.head.next,
				  struct file_map_history_item_s, list);
		list_del(&item->list);
		kfree(item);
	}
	spin_unlock(&file_map_history_list.lock);
}

static void file_map_history_show(uid_t uid)
{
	struct file_map_history_item_s *pos = NULL;
	unsigned long i = 0;

	/* -1 means show all uid's history */
	if (uid != -1 && !file_map_uid_valid(uid)) {
		fm_print(log_level_error,
			"file map write uid %d invalid.\n", uid);
		return;
	}

	fm_print(log_level_info, "[file map history]\n");
	spin_lock(&file_map_history_list.lock);
	list_for_each_entry(pos, &file_map_history_list.head, list) {
		if (uid == -1 || pos->uid == uid)
			fm_print(log_level_info,
			"[%-4lu] uid: %-10d, state: %d, res: %d, hitrate: %ld%%\n",
			i++, pos->uid, pos->state, pos->res, pos->hitrate);
	}
	spin_unlock(&file_map_history_list.lock);
}

/* file_map.s_lock hold */
static void file_map_data_show(void)
{
	struct file_map_entry *pentry = NULL;
	struct rtc_time tm;
	unsigned long i;
	unsigned long id;
	char *content = NULL;

	id = 0;
	fm_print(log_level_info, "[used list]:\n");
	list_for_each_entry(pentry, &file_map.used, list) {
		fm_print(log_level_info, "\n");
		fm_print(log_level_info, "ID:%lu\n", id++);
		fm_print(log_level_info, "uid:%d\n", pentry->uid);
		fm_print(log_level_info, "ino:%lu\n", pentry->ino);
		fm_print(log_level_info, "fs_type:%u\n", pentry->fs_type);
		fm_print(log_level_info, "size:%d\n", pentry->size);
		fm_print(log_level_info, "gen:%u\n", pentry->generation);
		fm_print(log_level_info, "kdev:%u\n", pentry->kdev);

		rtc_time_to_tm((unsigned long)pentry->mtime.tv_sec, &tm);
		fm_print(log_level_info,
			"mtime : %d-%02d-%02d %02d:%02d:%02d\n",
			tm.tm_year + 1900, tm.tm_mon + 1,
			tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		content = (char *)(pentry->maps);
		if (content) {
			for (i = 0; i < 32 && i < pentry->size; i += 8)
				fm_print(log_level_info,
				"[%02lu~%02lu] %02x %02x %02x %02x %02x %02x %02x %02x\n",
				i, i + 7,
				*(content + i), *(content + i + 1),
				*(content + i + 2), *(content + i + 3),
				*(content + i + 4), *(content + i + 5),
				*(content + i + 6), *(content + i + 7));
		}
	}

	fm_print(log_level_info, "[unused list]:\n");
	list_for_each_entry(pentry, &file_map.unused, list) {
		fm_print(log_level_info, "\n");
		fm_print(log_level_info, "ID:%lu\n", id++);
		fm_print(log_level_info, "uid:%d\n", pentry->uid);
		fm_print(log_level_info, "ino:%lu\n", pentry->ino);
		fm_print(log_level_info, "fs_type:%u\n", pentry->fs_type);
		fm_print(log_level_info, "size:%d\n", pentry->size);
		fm_print(log_level_info, "gen:%u\n", pentry->generation);
		fm_print(log_level_info, "kdev:%u\n", pentry->kdev);

		rtc_time_to_tm((unsigned long)pentry->mtime.tv_sec, &tm);
		fm_print(log_level_info,
			"mtime : %d-%02d-%02d %02d:%02d:%02d\n",
			tm.tm_year + 1900, tm.tm_mon + 1,
			tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		content = (char *)(pentry->maps);
		if (content) {
			for (i = 0; i < 32 && i < pentry->size; i += 8)
				fm_print(log_level_info,
				"[%02lu~%02lu] %02x %02x %02x %02x %02x %02x %02x %02x\n",
				i, i + 7,
				*(content + i), *(content + i + 1),
				*(content + i + 2), *(content + i + 3),
				*(content + i + 4), *(content + i + 5),
				*(content + i + 6), *(content + i + 7));
		}
	}
}
#endif

static int file_map_check_entry_outofdate(struct inode *inode,
			struct file_map_entry *pentry)
{
	uint32_t size = get_inode_bitmap_size(inode);

	if (size != pentry->size ||
		inode->i_generation != pentry->generation ||
		inode->i_sb->s_dev != pentry->kdev ||
		timespec_compare(&inode->i_mtime, &pentry->mtime) != 0) {
		fm_print(log_level_info,
			"uid %d ino %lu size %u/%u gen %u/%u kdev %u/%u mtime %ld.%ld/%ld.%ld bitmap entry out of date.\n",
			pentry->uid, inode->i_ino, size, pentry->size,
			inode->i_generation, pentry->generation,
			inode->i_sb->s_dev, pentry->kdev,
			inode->i_mtime.tv_sec, inode->i_mtime.tv_nsec,
			pentry->mtime.tv_sec, pentry->mtime.tv_nsec);
		return false;
	}

	return true;
}

/*
 * Function name: file_map_page_offset_update
 * Discription: Process mapping data based on access page offset
 * Parameters:
 *      @ struct file *filp
 *      @ pgoff_t offset: access start page offset
 *      @ pgoff_t last index: access last page offset(not included)
 * return value:
 *      @ NULL
 */
void file_map_page_offset_update(struct file *filp, pgoff_t offset,
				 pgoff_t last_index)
{
	unsigned long pgoff;
	struct file_map_entry *entry = NULL;
	struct inode *inode = NULL;
	unsigned long flags;

	if (!file_map_check_support())
		return;

	if (!filp)
		return;

	inode = filp->f_mapping->host;
	if (!inode)
		return;

	spin_lock_irqsave(&inode->i_lock, flags);
	entry = inode->i_file_map;

	if (!entry || !entry->size)
		goto out_unlock;

	if (!file_map_check_cur_uid(entry->uid))
		goto out_unlock;

	for (pgoff = offset; pgoff < last_index; pgoff++) {
		if (unlikely(pgoff >= (entry->size * BITS_PER_BYTE)))
			break;

		if (file_map_check_in_study()) {
			set_bit(pgoff, (unsigned long *)entry->maps);
		} else {
			if (test_bit(pgoff, (unsigned long *)entry->maps))
				file_map_hitrate_update_hit(inode);
			else
				file_map_hitrate_update_miss(inode);
		}
	}

out_unlock:
	spin_unlock_irqrestore(&inode->i_lock, flags);

}
EXPORT_SYMBOL(file_map_page_offset_update);

#define MAX_ANA_WINDWS_SIZE  512
static unsigned char file_map_check_data(struct file_map_entry *entry,
				pgoff_t offset, pgoff_t expand_end)
{
	unsigned char copy_bytes = 0;

	if (!entry || !entry->size)
		return 0;

	if (!file_map_check_cur_uid(entry->uid))
		return 0;

	if (offset / BITS_PER_BYTE >= entry->size)
		return 0;

	copy_bytes = (expand_end / BITS_PER_BYTE) -
				(offset / BITS_PER_BYTE) + 1;
	if ((offset / BITS_PER_BYTE) + copy_bytes > entry->size)
		copy_bytes = entry->size - (offset / BITS_PER_BYTE);

	copy_bytes = min_t(unsigned char, MAX_ANA_WINDWS_SIZE / BITS_PER_BYTE,
								copy_bytes);

	return copy_bytes;
}

unsigned long file_map_data_analysis(struct inode *inode,
					pgoff_t offset,
					unsigned long nr_to_read,
					unsigned long end_index,
					bool readaround)
{
	pgoff_t end;
	pgoff_t expand_end;
	unsigned int old_ra_pages = nr_to_read;
	unsigned int f_ra_pages = nr_to_read;
	struct file_map_entry *entry = NULL;
	uint32_t frags = 0;
	pgoff_t pos, new_pos;
	pgoff_t calc_end;
	unsigned long flags;
	/* ra windows max 256 pages, need buffer two ra windows */
	unsigned char pmap[MAX_ANA_WINDWS_SIZE / BITS_PER_BYTE];
	unsigned char copy_bytes = 0;

	if (!file_map_bitmap_enable)
		return f_ra_pages;

	/* fast judgement */
	if (!inode->i_file_map)
		return f_ra_pages;

#ifdef CONFIG_BLK_CGROUP
	if (!file_map_is_foreground()) {
		f_ra_pages = 0;
		atomic64_add(old_ra_pages - f_ra_pages,
			&file_map_stat.shrink_bg_pages);
		return f_ra_pages;
	}
#else
	return f_ra_pages;
#endif

	if (offset >= end_index || nr_to_read > (MAX_ANA_WINDWS_SIZE / 2))
		return f_ra_pages;

	end = min_t(pgoff_t, end_index, offset + nr_to_read);
	expand_end = min_t(pgoff_t, end_index, offset + 2 * nr_to_read);

	/* continuous bit1 is considered as one io */
	spin_lock_irqsave(&inode->i_lock, flags);
	entry = inode->i_file_map;
	copy_bytes = file_map_check_data(entry, offset, expand_end);
	if (copy_bytes == 0) {
		spin_unlock_irqrestore(&inode->i_lock, flags);
		return f_ra_pages;
	}

	memcpy(pmap, (unsigned char *)entry->maps + (offset / BITS_PER_BYTE),
		copy_bytes);
	spin_unlock_irqrestore(&inode->i_lock, flags);

	pos = offset % BITS_PER_BYTE;
	calc_end = min_t(pgoff_t, (copy_bytes * BITS_PER_BYTE) - 1,
				(end - offset) + offset % BITS_PER_BYTE);
	if (calc_end <= pos)
		return f_ra_pages;

	while (1) {
		pos = find_next_bit((unsigned long *)pmap, calc_end, pos);
		if (pos >= calc_end)
			break;

		frags++;
		if (frags > fm_frag_threshold) {
			f_ra_pages >>= 1;
			atomic64_add(old_ra_pages - f_ra_pages,
					&file_map_stat.shrink_fg_pages);
			return f_ra_pages;
		}

		pos = find_next_zero_bit((unsigned long *)pmap, calc_end, pos);
		if (pos >= calc_end)
			break;
	}

	if (!readaround) {
		if (frags == 1) {
			pos = calc_end;
			calc_end = min_t(pgoff_t,
				(copy_bytes * BITS_PER_BYTE) - 1,
				(expand_end - offset) + offset % BITS_PER_BYTE);
			if (calc_end > pos) {
				new_pos = find_next_zero_bit(
					(unsigned long *)pmap, calc_end, pos);

				f_ra_pages = min_t(unsigned long,
					f_ra_pages + new_pos - pos,
					2 * nr_to_read);
				atomic64_add(f_ra_pages - old_ra_pages,
					&file_map_stat.expand_fg_pages);
				atomic64_add(1,
					&file_map_stat.readforward_times);
			}
		}
	} else {
		atomic64_add(1, &file_map_stat.readaround_times);
		/* readaround not support expand windows now */
	}

	return f_ra_pages;
}
EXPORT_SYMBOL(file_map_data_analysis);

/*
 * Function name: file_map_ra_adapt
 * Discription: Check using io readahead
 * Parameters:
 *      @ void
 * return value:
 *      @ true: support io readahead
 */
bool file_map_ra_adapt(void)
{
	if (!file_map_bitmap_enable)
		return false;

	if (!file_map_check_in_using())
		return false;

	return true;
}
EXPORT_SYMBOL(file_map_ra_adapt);

static int file_map_proc_stats_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "ra pages: %ld %ld\n",
			atomic64_read(&file_map_stat.total_pages),
			atomic64_read(&file_map_stat.ignore_pages));
	seq_printf(seq, "shrink pages: %ld %ld\n",
			atomic64_read(&file_map_stat.shrink_bg_pages),
			atomic64_read(&file_map_stat.shrink_fg_pages));
	seq_printf(seq, "expand pages: %ld\n",
			atomic64_read(&file_map_stat.expand_fg_pages));
	seq_printf(seq, "frag threshold: %ld\n", fm_frag_threshold);
	seq_printf(seq, "lowhit threshold: %ld\n", fm_low_hitrate_threshold);
	seq_printf(seq, "hit stats: %ld %ld\n",
			atomic64_read(&file_map_stat.lowhit_times),
			atomic64_read(&file_map_stat.highhit_times));
	seq_printf(seq, "ra type stats: %ld %ld\n",
			atomic64_read(&file_map_stat.readforward_times),
			atomic64_read(&file_map_stat.readaround_times));

	return 0;
}

#ifdef FILE_MAP_DEBUG
static ssize_t file_map_proc_stats_write(struct file *filp,
			const char __user *buf, size_t count, loff_t *off)
{
	char kbuf[128];
	char arg0[8], arg1[8];
	long val;

	if (copy_from_user(kbuf, buf, 127))
		return -EINVAL;
	kbuf[127] = '\0';

	if (sscanf(kbuf, "%7s %7s", arg0, arg1) != 2)
		return -EINVAL;

	if (!strncmp(arg0, "-h", 2)) {
		if (!strncmp(arg1, "0", 1)) {
			file_map_history_show(-1);
		} else {
			if (kstrtol(arg1, 0, &val))
				return -EINVAL;
			file_map_history_show((uid_t)val);
		}
	} else if (!strncmp(arg0, "-r", 2)) {
		file_map_hitrate_show();
	} else {
		fm_print(log_level_info, "not support option %s\n", arg0);
	}

	return (ssize_t)count;
}
#endif

static int file_map_proc_stats_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, file_map_proc_stats_show, NULL);
}

static const struct file_operations file_map_proc_stats_fops = {
	.owner = THIS_MODULE,
	.open = file_map_proc_stats_open,
#ifdef FILE_MAP_DEBUG
	.write = file_map_proc_stats_write,
#endif
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static ssize_t file_map_read(struct file *filp, char __user *buf,
			     size_t count, loff_t *offp)
{
	uint32_t len = 0;
	char *map = NULL;
	ssize_t ret;

	if (!file_map_check_in_study()) {
		fm_print(log_level_error, "cur state not support.\n");
		return 0;
	}

	map = vmalloc(FILE_MAP_FILE_MAX_SIZE);
	if (!map) {
		fm_print(log_level_error, "alloc mem fail.\n");
		return -ENOMEM;
	}

	spin_lock(&file_map.s_lock);
#ifdef FILE_MAP_DEBUG
	file_map_data_show();
#endif
	len = file_map_study_content_extract(file_map.uid, map);
	spin_unlock(&file_map.s_lock);

	if (len < FILE_MAP_FILE_MIN_SIZE ||
		len > FILE_MAP_FILE_MAX_SIZE) {
		fm_print(log_level_error, "study list len %u invalid.\n", len);
		ret = -EFAULT;
		goto out;
	}

	if (count < len) {
		fm_print(log_level_error,
			"buf %zu(%u) too small.\n", count, len);
		ret = -EFAULT;
		goto out;
	}

	if (copy_to_user(buf, map, len)) {
		fm_print(log_level_error, "copy to user fail.\n");
		ret = -EFAULT;
		goto out;
	}

	fm_print(log_level_info, "read %u ok.\n", len);
	ret = (ssize_t)len;
out:
	vfree(map);
	return ret;
}

/* hold file_map.s_lock */
static void file_map_entry_add_inode(struct file_map_entry *entry,
				     struct inode *inode)
{
	spin_lock(&inode->i_lock);
	inode->i_file_map = entry;
	spin_unlock(&inode->i_lock);

	entry->inode = inode;
	list_move(&entry->list, &file_map.used);
}

void file_map_entry_del_inode(struct inode *inode)
{
	struct file_map_entry *entry = NULL;

	if (!file_map_bitmap_enable)
		return;

	spin_lock(&file_map.s_lock);
	spin_lock(&inode->i_lock);
	entry = inode->i_file_map;
	inode->i_file_map = NULL;
	spin_unlock(&inode->i_lock);

	if (!entry) {
		spin_unlock(&file_map.s_lock);
		return;
	}

	entry->inode = NULL;
	list_move(&entry->list, &file_map.unused);
	spin_unlock(&file_map.s_lock);
}
EXPORT_SYMBOL(file_map_entry_del_inode);

/* hold file_map.s_lock */
static void __file_map_entry_del_inode(struct file_map_entry *entry)
{
	struct inode *inode = entry->inode;

	spin_lock(&inode->i_lock);
	inode->i_file_map = NULL;
	spin_unlock(&inode->i_lock);

	entry->inode = NULL;
	list_move(&entry->list, &file_map.unused);
}

static void file_map_entry_free(struct file_map_entry *entry)
{
	list_del(&entry->list);
	entry->size = 0;
	kfree(entry);
}

/* hold file_map.s_lock */
static void file_map_entry_clear(void)
{
	struct file_map_entry *entry = NULL;
	struct file_map_entry *tmp = NULL;

	list_for_each_entry_safe(entry, tmp, &file_map.used, list)
		__file_map_entry_del_inode(entry);
	list_for_each_entry_safe(entry, tmp, &file_map.unused, list)
		file_map_entry_free(entry);
}

static struct file_map_entry *file_map_entry_alloc(struct bitmap_hdr *hdr,
						   uid_t uid)
{
	struct file_map_entry *entry;
	uint32_t size = le32_to_cpu(hdr->size);

	entry = kmalloc(sizeof(*entry) + size, GFP_KERNEL);
	if (!entry)
		return NULL;

	INIT_LIST_HEAD(&entry->list);
	entry->inode = NULL;
	entry->uid = uid;
	entry->ino = le32_to_cpu(hdr->ino);
	entry->size = size;
	entry->fs_type = le32_to_cpu(hdr->fs_type);
	entry->mtime.tv_sec = le64_to_cpu(hdr->mtime);
	entry->mtime.tv_nsec = le32_to_cpu(hdr->mtime_nsec);
	entry->generation = le32_to_cpu(hdr->generation);
	entry->kdev = le32_to_cpu(hdr->kdev);
	memcpy(entry->maps, (char *)hdr + sizeof(*hdr), size);

	spin_lock(&file_map.s_lock);
	list_add_tail(&entry->list, &file_map.unused);
	spin_unlock(&file_map.s_lock);

	return entry;
}

void file_map_entry_attach_unused(struct inode *inode)
{
	struct file_map_entry *entry = NULL;
	struct file_map_entry *tmp = NULL;

	if (!file_map_bitmap_enable)
		return;

	if (!file_map_check_in_using())
		return;

	if (file_map.res != FILE_MAP_RES_USING_BITMAP_OK)
		return;

	spin_lock(&file_map.s_lock);
	list_for_each_entry_safe(entry, tmp, &file_map.unused, list) {
		if (entry->ino != inode->i_ino)
			continue;

		if (!file_map_check_entry_outofdate(inode, entry))
			continue;

		file_map_entry_add_inode(entry, inode);
		break;
	}
	spin_unlock(&file_map.s_lock);
}
EXPORT_SYMBOL(file_map_entry_attach_unused);

bool file_map_is_set(struct inode *inode, pgoff_t page_offset)
{
	struct file_map_entry *entry = NULL;
	bool ret = false;
	unsigned long flags;

	if (!file_map_bitmap_enable)
		return true;

	spin_lock_irqsave(&inode->i_lock, flags);
	entry = inode->i_file_map;
	if (entry &&/* entry->maps && */entry->size)
		ret = test_bit(page_offset, entry->maps);
	else
		ret = true;
	spin_unlock_irqrestore(&inode->i_lock, flags);

	return ret;
}
EXPORT_SYMBOL(file_map_is_set);

static void file_map_study_entry_alloc(struct inode *inode,
					enum file_map_fs_type fs_type)
{
	struct file_map_entry *entry = NULL;
	struct file_map_entry *tmp = NULL;
	bool in_unused = false;
	uint32_t size;

	if (inode->i_file_map)
		return;

	size = get_inode_bitmap_size(inode);

	spin_lock(&file_map.s_lock);
	if (!file_map_check_cur_uid(file_map.uid)) {
		spin_unlock(&file_map.s_lock);
		return;
	}

	list_for_each_entry_safe(entry, tmp, &file_map.unused, list) {
		if (entry->ino != inode->i_ino)
			continue;

		if (!file_map_check_entry_outofdate(inode, entry))
			continue;

		file_map_entry_add_inode(entry, inode);
		in_unused = true;
		break;
	}

	if (in_unused || file_map.nr_bitmaps >= FILE_MAP_MAX_FILES_PER_UID) {
		spin_unlock(&file_map.s_lock);
		return;
	}
	spin_unlock(&file_map.s_lock);

	entry = kzalloc(sizeof(*entry) + size, GFP_KERNEL);
	if (!entry)
		return;

	spin_lock(&file_map.s_lock);
	INIT_LIST_HEAD(&entry->list);
	entry->uid = file_map.uid;
	entry->ino = inode->i_ino;
	entry->inode = inode;
	entry->size = size;
	entry->fs_type = fs_type;
	entry->mtime = inode->i_mtime;
	entry->generation = inode->i_generation;
	entry->kdev = inode->i_sb->s_dev;

	spin_lock(&inode->i_lock);
	inode->i_file_map = entry;
	spin_unlock(&inode->i_lock);

	list_add_tail(&entry->list, &file_map.used);
	file_map.nr_bitmaps++;
	spin_unlock(&file_map.s_lock);
}

/*
 * Function name: file_map_filter
 * Discription: Filter the file to determine if it meets the conditions for the
 *              file map activity
 * Parameters:
 *      @ struct file *filp
 *      @ const char *name: file name str
 * return value:
 *      @ NULL
 */
void file_map_filter(struct file *filp, const char *name)
{
	struct inode *inode = NULL;
	enum file_map_fs_type fs_type;

	if (!filp || !name)
		return;

	inode = filp->f_inode;

	if (!file_map_bitmap_enable)
		return;

	if (!file_map_check_in_study())
		return;

	if (!file_map_file_prefilter(filp, name, &fs_type))
		return;

	file_map_study_entry_alloc(inode, fs_type);
}
EXPORT_SYMBOL(file_map_filter);

static struct super_block *file_map_get_super(dev_t dev)
{
	struct super_block *sb = NULL;
	struct block_device *bdev = bdget(dev);

	if (bdev) {
		sb = get_super(bdev);
		bdput(bdev);
	}

	return sb;
}

extern struct inode *find_inode_fast_ext(struct super_block *sb,
					unsigned long ino);
static int file_map_bitmap_file_push(uid_t uid,
			size_t count, struct bitmap_file_hdr *pfhdr, char *map)
{
	unsigned long i;
	int ret = 0;
	struct bitmap_hdr *pbhdr = NULL;
	uint32_t nr_bitmaps;
	uint32_t cur_offset = 0;
	uint32_t last_end = 0;
	uint32_t b_magic;
	struct file_map_entry *entry = NULL;
	struct super_block *sb = NULL;
	struct inode *inode = NULL;
	uint32_t s_offset, e_offset, b_size, b_offset;

	nr_bitmaps = le32_to_cpu(pfhdr->nr_bitmaps);
	last_end = FILE_MAP_FILE_HEAD_LEN;

	for (i = 0; i < nr_bitmaps; i++) {
		cur_offset = le32_to_cpu(pfhdr->bitmaps_header_offset[i]);
		if (cur_offset > count ||
		    (cur_offset + FILE_MAP_HEAD_LEN) > count) {
			fm_print(log_level_error,
				"idx %lu offset %d invalid.\n", i, cur_offset);
			ret = -EINVAL;
			goto err_out;
		}

		pbhdr = (struct bitmap_hdr *)(map + cur_offset);
		b_magic = le32_to_cpu(pbhdr->magic);
		if (b_magic != FILE_MAP_MAPHEAD_MAGIC) {
			fm_print(log_level_error, "idx %lu magic %d invalid.\n",
				i, b_magic);
			ret = -EINVAL;
			goto err_out;
		}

		s_offset = le32_to_cpu(pbhdr->start_file_offset);
		e_offset = le32_to_cpu(pbhdr->end_file_offset);
		b_offset = le32_to_cpu(pbhdr->bitmap_offset);
		b_size = le32_to_cpu(pbhdr->size);

		if (s_offset != cur_offset || s_offset != last_end ||
		    e_offset > count ||
		    s_offset + FILE_MAP_HEAD_LEN != b_offset ||
		    b_offset + b_size != e_offset ||
		    b_size > FILE_MAP_FILE_SINGLE_SIZE_MAX) {
			fm_print(log_level_error,
				"idx %lu s_offset %d e_offset %d b_offset %d b_size %d invalid.\n",
				i, s_offset, e_offset, b_offset, b_size);
			ret = -EINVAL;
			goto err_out;
		}

		last_end = le32_to_cpu(pbhdr->end_file_offset);

		entry = file_map_entry_alloc(pbhdr, uid);
		if (!entry) {
			fm_print(log_level_info,
				"ino %d allcoc entry mem fail.\n",
				le32_to_cpu(pbhdr->ino));
			continue;
		}

		sb = file_map_get_super(le32_to_cpu(pbhdr->kdev));
		if (!sb) {
			fm_print(log_level_info, "get sb fail.\n");
			continue;
		}

		inode = find_inode_fast_ext(sb, le32_to_cpu(pbhdr->ino));
		if (inode) {
			if (le32_to_cpu(pbhdr->ino) != inode->i_ino) {
				fm_print(log_level_info,
					"ino %u/%lu not match.\n",
					le32_to_cpu(pbhdr->ino), inode->i_ino);
				iput(inode);
				drop_super(sb);
				continue;
			}

			if (!file_map_check_entry_outofdate(inode, entry)) {
				iput(inode);
				drop_super(sb);
				continue;
			}

			spin_lock(&file_map.s_lock);
			file_map_entry_add_inode(entry, inode);
			spin_unlock(&file_map.s_lock);

			iput(inode);
		}
		drop_super(sb);
		file_map_hitrate_set_ino(i, le32_to_cpu(pbhdr->ino));
	}

	ret = i;

err_out:
	if (ret < 0) {
		file_map_hitrate_clear();
		spin_lock(&file_map.s_lock);
		file_map.nr_bitmaps = 0;
		file_map_entry_clear();
		spin_unlock(&file_map.s_lock);
	}

	return ret;

}

static ssize_t file_map_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *offp)
{
	struct bitmap_file_hdr *pfhdr = NULL;
	uint32_t nr_bitmaps, f_magic;
	__wsum csum;
	char *map = NULL;
	size_t ret;
	uid_t uid;

	if (!file_map_check_in_using())
		return 0;

	if (count > FILE_MAP_FILE_MAX_SIZE || count < FILE_MAP_FILE_MIN_SIZE) {
		fm_print(log_level_error, "size %lu not in scope [%ld %ld].\n",
			count, FILE_MAP_FILE_MIN_SIZE, FILE_MAP_FILE_MAX_SIZE);
		return -EINVAL;
	}

	map = memdup_user(buf, count);
	if (IS_ERR(map)) {
		fm_print(log_level_error, "memdup user fail.\n");
		return PTR_ERR(map);
	}

	pfhdr = (struct bitmap_file_hdr *)map;
	uid = le32_to_cpu(pfhdr->uid);
	f_magic = le32_to_cpu(pfhdr->magic);
	nr_bitmaps = le32_to_cpu(pfhdr->nr_bitmaps);

	csum = csum_partial(map + sizeof(__le32), count - sizeof(__le32), 0);
	if (csum != le32_to_cpu(pfhdr->crc)) {
		fm_print(log_level_error, "crc check invalid.\n");
		ret = -EINVAL;
		goto err_out;
	}

	if (!file_map_uid_valid(uid)) {
		fm_print(log_level_error, "uid %d invalid.\n", uid);
		ret = -EINVAL;
		goto err_out;
	}

	if (f_magic != FILE_MAP_FILE_MAGIC ||
	    nr_bitmaps <= 0 || nr_bitmaps > FILE_MAP_MAX_FILES_PER_UID) {
		fm_print(log_level_error, "magic %d nr %d invalid.\n",
			f_magic, nr_bitmaps);
		ret = -EINVAL;
		goto err_out;
	}

	ret = file_map_bitmap_file_push(uid, count, pfhdr, map);
	if (ret < 0)
		goto err_out;

	spin_lock(&file_map.s_lock);
	file_map.nr_bitmaps = ret;
	file_map.res = FILE_MAP_RES_USING_BITMAP_OK;
	spin_unlock(&file_map.s_lock);

#ifdef FILE_MAP_DEBUG
	file_map_data_show();
#endif

	fm_print(log_level_info, "write %zu bytes ok.\n", count);
	kfree(map);

	return count;

err_out:
	spin_lock(&file_map.s_lock);
	file_map.nr_bitmaps = 0;
	file_map.res = FILE_MAP_RES_USING_BITMAP_INVALID;
	spin_unlock(&file_map.s_lock);

	kfree(map);

	return ret;
}

/* hold file_map.s_lock */
static void file_map_data_reinit(uid_t uid,
				enum file_map_state state)
{
	file_map.state = state;
	file_map.uid = uid;
	file_map.nr_bitmaps = 0;
	file_map.res = FILE_MAP_RES_MAX;
	file_map_entry_clear();
}

static void file_map_timeout(unsigned long data)
{
	file_map_bitmap_enable = false;

	spin_lock(&file_map.s_lock);
	file_map_data_reinit(-1, FILE_MAP_STATE_DISABLE);
	spin_unlock(&file_map.s_lock);

	fm_print(log_level_info, "timeout to end io readahead.\n");
}

static void file_map_add_timer(void)
{
	spin_lock(&file_map.s_lock);
	mod_timer(&file_map.timer, jiffies + 30 * HZ);
	spin_unlock(&file_map.s_lock);
}

static void file_map_del_timer(void)
{
	spin_lock(&file_map.s_lock);
	del_timer(&file_map.timer);
	spin_unlock(&file_map.s_lock);
}

static int file_map_app_launch_begin(unsigned long arg)
{
	struct app_cold_launch_begin_para para;
	uid_t uid;
	uint32_t state;

	if (copy_from_user(&para,
			   (struct app_cold_launch_begin_para __user *)arg,
			   sizeof(para)))
		return -EFAULT;

	if (para.state >= FILE_MAP_STATE_MAX || !file_map_uid_valid(para.uid)) {
		fm_print(log_level_error, "invalid arg, uid[%d] state[%d]\n",
			para.uid, para.state);
		return -EINVAL;
	}

	spin_lock(&file_map.s_lock);
	file_map_data_reinit(para.uid, para.state);
	uid = file_map.uid;
	state = file_map.state;
	file_map_hitrate_clear();
	spin_unlock(&file_map.s_lock);

	fm_print(log_level_info, "launch begin uid[%d] state[%d]\n",
		uid, state);

	return 0;
}

static int file_map_app_launch_end(unsigned long arg)
{
	uid_t uid; /* future use */

	if (copy_from_user(&uid, (uint32_t __user *)arg, sizeof(uint32_t)))
		return -EFAULT;

	if (!file_map_uid_valid(uid)) {
		fm_print(log_level_error, "invalid uid[%d]\n", uid);
		return -EINVAL;
	}

	spin_lock(&file_map.s_lock);
	file_map_data_reinit(-1, FILE_MAP_STATE_DISABLE);
	spin_unlock(&file_map.s_lock);

	fm_print(log_level_info, "launch end uid[%d]\n", uid);

	return 0;
}

static int file_map_app_get_res(unsigned long arg)
{
	uid_t uid;
	long hitrate = 0;
	enum file_map_state state;
	enum file_map_result res;
#ifdef FILE_MAP_DEBUG
	struct file_map_history_item_s *his = NULL;
#endif

	spin_lock(&file_map.s_lock);
	if (file_map.state == FILE_MAP_STATE_STUDY) {
		if (list_empty(&file_map.used) && list_empty(&file_map.unused))
			file_map.res = FILE_MAP_RES_STUDY_NO_FILE;
		else
			file_map.res = FILE_MAP_RES_STUDY_COMPLETE;
	} else if (file_map.state == FILE_MAP_STATE_USING) {
		if (file_map.res == FILE_MAP_RES_USING_BITMAP_OK) {
			/*
			 * Map data is used normally,
			 * then check usage statistics.
			 * if uid total hitrate less than threshold
			 * is judged to be invalid.
			 * Tips: only support study in uid units,
			 * do not support signal file study.
			 */
			hitrate = file_map_get_hitrate();
			if (hitrate < fm_low_hitrate_threshold) {
				file_map.res =
				FILE_MAP_RES_USING_BITMAP_LOWHIT;
				atomic64_add(1,
					&file_map_stat.lowhit_times);
			} else {
				atomic64_add(1,
					&file_map_stat.highhit_times);
			}
		}
	} else {
		file_map.res = FILE_MAP_RES_FUN_NOT_ENABLE;
	}

	res = file_map.res;
	uid = file_map.uid;
	state = file_map.state;
	spin_unlock(&file_map.s_lock);

	if (copy_to_user((void __user *)arg, &res, sizeof(uint32_t)))
		return -EFAULT;

#ifdef FILE_MAP_DEBUG
	his = kzalloc(sizeof(struct file_map_history_item_s),
					GFP_KERNEL);
	if (his) {
		his->uid = uid;
		his->state = state;
		his->res = res;
		his->hitrate = hitrate;
		file_map_history_add(his);
	}

	if (state == FILE_MAP_STATE_USING) {
		file_map_history_show(-1);
		file_map_hitrate_show();
	}
#endif

	fm_print(log_level_info, "get result: %d.\n", res);

	return 0;
}

static long file_map_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int error = 0;

	if (_IOC_TYPE(cmd) != FILE_MAP_IOC_MAGIC) {
		fm_print(log_level_error, "%d magic invalid %d.\n",
				cmd, FILE_MAP_IOC_MAGIC);
		return -EINVAL;
	}

	if (_IOC_NR(cmd) > FILE_MAP_IOC_MAXNR) {
		fm_print(log_level_error, "%d larger than max cmd number %d.\n",
					cmd, FILE_MAP_IOC_MAXNR);
		return -EINVAL;
	}

	switch (cmd) {
	case FILE_MAP_ENABLE_CTL: {
		bool enable = false;

		if (copy_from_user(&enable, (bool __user *)arg, sizeof(bool)))
			return -EFAULT;

		if (enable)
			file_map_add_timer();
		else
			file_map_del_timer();

		file_map_bitmap_enable = enable;

		fm_print(log_level_info, "set file_map_bitmap_enable %d.\n",
					file_map_bitmap_enable);
		break;
	}
	case FILE_MAP_COLD_LAUNCH_BEGIN:
		error = file_map_app_launch_begin(arg);
		break;
	case FILE_MAP_COLD_LAUNCH_END:
		error = file_map_app_launch_end(arg);
		break;
	case FILE_MAP_GET_RESULT:
		error = file_map_app_get_res(arg);
		break;
	default:
		fm_print(log_level_info, "not support cmd %d\n", cmd);
		error = -ENOTTY;
	}

	return error;
}

static const struct file_operations file_map_fops = {
	.read = file_map_read,
	.write = file_map_write,
	.compat_ioctl = file_map_ioctl,
	.unlocked_ioctl = file_map_ioctl,
};

static struct miscdevice file_map_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "file_map",
	.fops = &file_map_fops,
};

#define FILE_MAP_PROC_ENTRY	"driver/file_map_status"
static int __init file_map_init(void)
{
	struct proc_dir_entry *entry = NULL;

	setup_timer(&file_map.timer, file_map_timeout, 0);

	memset(&file_map_stat, 0, sizeof(file_map_stat));
	memset(&hitrate_stat, 0, sizeof(hitrate_stat));

	entry = proc_create(FILE_MAP_PROC_ENTRY, 0600, NULL,
			    &file_map_proc_stats_fops);
	if (!entry) {
		fm_print(log_level_error, "create proc entry fail\n");
		return -ENOMEM;
	}

	if (misc_register(&file_map_device)) {
		remove_proc_entry(FILE_MAP_PROC_ENTRY, NULL);
		fm_print(log_level_error, "register device fail\n");
		return -ENOMEM;
	}

	fm_print(log_level_info, "file map module init ok\n");

	return 0;
};

static void __exit file_map_exit(void)
{
	misc_deregister(&file_map_device);
	remove_proc_entry(FILE_MAP_PROC_ENTRY, NULL);
	del_timer(&file_map.timer);
};

module_init(file_map_init);
module_exit(file_map_exit);

MODULE_AUTHOR("os kernel lab, HUAWEI Inc.");
MODULE_DESCRIPTION("readahead file map");
MODULE_LICENSE("GPL");

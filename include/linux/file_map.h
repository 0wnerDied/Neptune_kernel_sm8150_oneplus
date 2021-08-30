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
 * File Name:    file_map.h
 *
 * Description:
 */

#ifndef _FILE_MAP_H
#define _FILE_MAP_H
#include <linux/types.h>

#define CONFIG_CGROUP_IOLIMIT_IN_FILE_PAGEFAULT 1  /* iolimit */
//#define FILE_MAP_DEBUG  1

void file_map_stat_total_inc(long cnt);
void file_map_stat_ignore_inc(long cnt);
void file_map_entry_del_inode(struct inode *inode);
void file_map_filter(struct file *filp, const char *name);
bool file_map_ra_adapt(void);
void file_map_page_offset_update(struct file *filp,
					pgoff_t offset, pgoff_t last_index);
void file_map_entry_attach_unused(struct inode *inode);
bool file_map_is_set(struct inode *inode, pgoff_t page_offset);
unsigned long file_map_data_analysis(struct inode *inode,
					pgoff_t offset,
					unsigned long nr_to_read,
					unsigned long end_index,
					bool readaround);

#endif /*  _FILE_MAP_H */



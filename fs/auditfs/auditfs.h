/*
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009	   Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 * Copyright (c) 2012      Kenneth Ko
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _AUDITFS_H_
#define _AUDITFS_H_

#include <linux/dcache.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/seq_file.h>
#include <linux/statfs.h>
#include <linux/fs_stack.h>
#include <linux/magic.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sched.h>

/* the file system name */
#define AUDITFS_NAME "auditfs"

/* auditfs root inode number */
#define AUDITFS_ROOT_INO     1

/* useful for tracking code reachability */
#define UDBG printk(KERN_DEFAULT "DBG:%s:%s:%d\n", __FILE__, __func__, __LINE__)

/* operations vectors defined in specific files */
extern const struct file_operations auditfs_main_fops;
extern const struct file_operations auditfs_dir_fops;
extern const struct inode_operations auditfs_main_iops;
extern const struct inode_operations auditfs_dir_iops;
extern const struct inode_operations auditfs_symlink_iops;
extern const struct super_operations auditfs_sops;
extern const struct dentry_operations auditfs_dops;
extern const struct address_space_operations auditfs_aops, auditfs_dummy_aops;
extern const struct vm_operations_struct auditfs_vm_ops;

extern int auditfs_init_inode_cache(void);
extern void auditfs_destroy_inode_cache(void);
extern int auditfs_init_dentry_cache(void);
extern void auditfs_destroy_dentry_cache(void);
extern int new_dentry_private_data(struct dentry *dentry);
extern void free_dentry_private_data(struct dentry *dentry);
extern struct dentry *auditfs_lookup(struct inode *dir, struct dentry *dentry,
				    struct nameidata *nd);
extern int auditfs_interpose(struct dentry *dentry, struct super_block *sb,
			    struct path *lower_path);

/* file private data */
struct auditfs_file_info {
	struct file *lower_file;
	const struct vm_operations_struct *lower_vm_ops;
};

/* auditfs inode data in memory */
struct auditfs_inode_info {
	struct inode *lower_inode;
	struct inode vfs_inode;
};

/* auditfs dentry data in memory */
struct auditfs_dentry_info {
	spinlock_t lock;	/* protects lower_path */
	struct path lower_path;
};

/* auditfs super-block data in memory */
struct auditfs_sb_info {
	struct super_block *lower_sb;
};

/*
 * inode to private data
 *
 * Since we use containers and the struct inode is _inside_ the
 * auditfs_inode_info structure, AUDITFS_I will always (given a non-NULL
 * inode pointer), return a valid non-NULL pointer.
 */
static inline struct auditfs_inode_info *AUDITFS_I(const struct inode *inode)
{
	return container_of(inode, struct auditfs_inode_info, vfs_inode);
}

/* dentry to private data */
#define AUDITFS_D(dent) ((struct auditfs_dentry_info *)(dent)->d_fsdata)

/* superblock to private data */
#define AUDITFS_SB(super) ((struct auditfs_sb_info *)(super)->s_fs_info)

/* file to private Data */
#define AUDITFS_F(file) ((struct auditfs_file_info *)((file)->private_data))

/* file to lower file */
static inline struct file *auditfs_lower_file(const struct file *f)
{
	return AUDITFS_F(f)->lower_file;
}

static inline void auditfs_set_lower_file(struct file *f, struct file *val)
{
	AUDITFS_F(f)->lower_file = val;
}

/* inode to lower inode. */
static inline struct inode *auditfs_lower_inode(const struct inode *i)
{
	return AUDITFS_I(i)->lower_inode;
}

static inline void auditfs_set_lower_inode(struct inode *i, struct inode *val)
{
	AUDITFS_I(i)->lower_inode = val;
}

/* superblock to lower superblock */
static inline struct super_block *auditfs_lower_super(
	const struct super_block *sb)
{
	return AUDITFS_SB(sb)->lower_sb;
}

static inline void auditfs_set_lower_super(struct super_block *sb,
					  struct super_block *val)
{
	AUDITFS_SB(sb)->lower_sb = val;
}

/* path based (dentry/mnt) macros */
static inline void pathcpy(struct path *dst, const struct path *src)
{
	dst->dentry = src->dentry;
	dst->mnt = src->mnt;
}
/* Returns struct path.  Caller must path_put it. */
static inline void auditfs_get_lower_path(const struct dentry *dent,
					 struct path *lower_path)
{
	spin_lock(&AUDITFS_D(dent)->lock);
	pathcpy(lower_path, &AUDITFS_D(dent)->lower_path);
	path_get(lower_path);
	spin_unlock(&AUDITFS_D(dent)->lock);
	return;
}
static inline void auditfs_put_lower_path(const struct dentry *dent,
					 struct path *lower_path)
{
	path_put(lower_path);
	return;
}
static inline void auditfs_set_lower_path(const struct dentry *dent,
					 struct path *lower_path)
{
	spin_lock(&AUDITFS_D(dent)->lock);
	pathcpy(&AUDITFS_D(dent)->lower_path, lower_path);
	spin_unlock(&AUDITFS_D(dent)->lock);
	return;
}
static inline void auditfs_reset_lower_path(const struct dentry *dent)
{
	spin_lock(&AUDITFS_D(dent)->lock);
	AUDITFS_D(dent)->lower_path.dentry = NULL;
	AUDITFS_D(dent)->lower_path.mnt = NULL;
	spin_unlock(&AUDITFS_D(dent)->lock);
	return;
}
static inline void auditfs_put_reset_lower_path(const struct dentry *dent)
{
	struct path lower_path;
	spin_lock(&AUDITFS_D(dent)->lock);
	pathcpy(&lower_path, &AUDITFS_D(dent)->lower_path);
	AUDITFS_D(dent)->lower_path.dentry = NULL;
	AUDITFS_D(dent)->lower_path.mnt = NULL;
	spin_unlock(&AUDITFS_D(dent)->lock);
	path_put(&lower_path);
	return;
}

/* locking helpers */
static inline struct dentry *lock_parent(struct dentry *dentry)
{
	struct dentry *dir = dget_parent(dentry);
	mutex_lock_nested(&dir->d_inode->i_mutex, I_MUTEX_PARENT);
	return dir;
}

static inline void unlock_dir(struct dentry *dir)
{
	mutex_unlock(&dir->d_inode->i_mutex);
	dput(dir);
}
#endif	/* not _AUDITFS_H_ */

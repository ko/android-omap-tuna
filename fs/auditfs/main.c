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

#include "auditfs.h"
#include <linux/module.h>

/*
 * our custom d_alloc_root work-alike
 *
 * we can't use d_alloc_root if we want to use our own interpose function
 * unchanged, so we simply call our own "fake" d_alloc_root
 */
static struct dentry *auditfs_d_alloc_root(struct super_block *sb)
{
	struct dentry *ret = NULL;

	if (sb) {
		static const struct qstr name = {
			.name = "/",
			.len = 1
		};

		ret = d_alloc(NULL, &name);
		if (ret) {
			d_set_d_op(ret, &auditfs_dops);
			ret->d_sb = sb;
			ret->d_parent = ret;
		}
	}
	return ret;
}

/*
 * There is no need to lock the auditfs_super_info's rwsem as there is no
 * way anyone can have a reference to the superblock at this point in time.
 */
static int auditfs_read_super(struct super_block *sb, void *raw_data, int silent)
{
	int err = 0;
	struct super_block *lower_sb;
	struct path lower_path;
	char *dev_name = (char *) raw_data;

	if (!dev_name) {
		printk(KERN_ERR
		       "auditfs: read_super: missing dev_name argument\n");
		err = -EINVAL;
		goto out;
	}

	/* parse lower path */
	err = kern_path(dev_name, LOOKUP_FOLLOW | LOOKUP_DIRECTORY,
			&lower_path);
	if (err) {
		printk(KERN_ERR	"auditfs: error accessing "
		       "lower directory '%s'\n", dev_name);
		goto out;
	}

	/* allocate superblock private data */
	sb->s_fs_info = kzalloc(sizeof(struct auditfs_sb_info), GFP_KERNEL);
	if (!AUDITFS_SB(sb)) {
		printk(KERN_CRIT "auditfs: read_super: out of memory\n");
		err = -ENOMEM;
		goto out_free;
	}

	/* set the lower superblock field of upper superblock */
	lower_sb = lower_path.dentry->d_sb;
	atomic_inc(&lower_sb->s_active);
	auditfs_set_lower_super(sb, lower_sb);

	/* inherit maxbytes from lower file system */
	sb->s_maxbytes = lower_sb->s_maxbytes;

	/*
	 * Our c/m/atime granularity is 1 ns because we may stack on file
	 * systems whose granularity is as good.
	 */
	sb->s_time_gran = 1;

	sb->s_op = &auditfs_sops;

	/* see comment next to the definition of auditfs_d_alloc_root */
	sb->s_root = auditfs_d_alloc_root(sb);
	if (!sb->s_root) {
		err = -ENOMEM;
		goto out_sput;
	}

	/* link the upper and lower dentries */
	sb->s_root->d_fsdata = NULL;
	err = new_dentry_private_data(sb->s_root);
	if (err)
		goto out_freeroot;

	/* set the lower dentries for s_root */
	auditfs_set_lower_path(sb->s_root, &lower_path);

	/* call interpose to create the upper level inode */
	err = auditfs_interpose(sb->s_root, sb, &lower_path);
	if (!err) {
		if (!silent)
			printk(KERN_INFO
			       "auditfs: mounted on top of %s type %s\n",
			       dev_name, lower_sb->s_type->name);
		goto out;
	}
	/* else error: fall through */

	free_dentry_private_data(sb->s_root);
out_freeroot:
	dput(sb->s_root);
out_sput:
	/* drop refs we took earlier */
	atomic_dec(&lower_sb->s_active);
	kfree(AUDITFS_SB(sb));
	sb->s_fs_info = NULL;
out_free:
	path_put(&lower_path);

out:
	return err;
}

struct dentry *auditfs_mount(struct file_system_type *fs_type, int flags,
			    const char *dev_name, void *raw_data)
{
	void *lower_path_name = (void *) dev_name;

	return mount_nodev(fs_type, flags, lower_path_name,
			   auditfs_read_super);
}

static struct file_system_type auditfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= AUDITFS_NAME,
	.mount		= auditfs_mount,
	.kill_sb	= generic_shutdown_super,
	.fs_flags	= FS_REVAL_DOT,
};

static int __init init_auditfs_fs(void)
{
	int err;

	pr_info("Registering auditfs " AUDITFS_VERSION "\n");

	err = auditfs_init_inode_cache();
	if (err)
		goto out;
	err = auditfs_init_dentry_cache();
	if (err)
		goto out;
	err = register_filesystem(&auditfs_fs_type);
out:
	if (err) {
		auditfs_destroy_inode_cache();
		auditfs_destroy_dentry_cache();
	}
	return err;
}

static void __exit exit_auditfs_fs(void)
{
	auditfs_destroy_inode_cache();
	auditfs_destroy_dentry_cache();
	unregister_filesystem(&auditfs_fs_type);
	pr_info("Completed auditfs module unload\n");
}

MODULE_AUTHOR("Erez Zadok, Filesystems and Storage Lab, Stony Brook University"
	      " (http://www.fsl.cs.sunysb.edu/)");
MODULE_DESCRIPTION("auditfs " auditfs_VERSION
		   " (http://auditfs.filesystems.org/)");
MODULE_LICENSE("GPL");

module_init(init_auditfs_fs);
module_exit(exit_auditfs_fs);

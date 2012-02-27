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

/*
 * The inode cache is used with alloc_inode for both our inode info and the
 * vfs inode.
 */
static struct kmem_cache *auditfs_inode_cachep;

/* final actions when unmounting a file system */
static void auditfs_put_super(struct super_block *sb)
{
	struct auditfs_sb_info *spd;
	struct super_block *s;

	spd = AUDITFS_SB(sb);
	if (!spd)
		return;

	/* decrement lower super references */
	s = auditfs_lower_super(sb);
	auditfs_set_lower_super(sb, NULL);
	atomic_dec(&s->s_active);

	kfree(spd);
	sb->s_fs_info = NULL;
}

static int auditfs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	int err;
	struct path lower_path;

	auditfs_get_lower_path(dentry, &lower_path);
	err = vfs_statfs(&lower_path, buf);
	auditfs_put_lower_path(dentry, &lower_path);

	/* set return buf to our f/s to avoid confusing user-level utils */
	buf->f_type = AUDITFS_SUPER_MAGIC;

	return err;
}

/*
 * @flags: numeric mount options
 * @options: mount options string
 */
static int auditfs_remount_fs(struct super_block *sb, int *flags, char *options)
{
	int err = 0;

	/*
	 * The VFS will take care of "ro" and "rw" flags among others.  We
	 * can safely accept a few flags (RDONLY, MANDLOCK), and honor
	 * SILENT, but anything else left over is an error.
	 */
	if ((*flags & ~(MS_RDONLY | MS_MANDLOCK | MS_SILENT)) != 0) {
		printk(KERN_ERR
		       "auditfs: remount flags 0x%x unsupported\n", *flags);
		err = -EINVAL;
	}

	return err;
}

/*
 * Called by iput() when the inode reference count reached zero
 * and the inode is not hashed anywhere.  Used to clear anything
 * that needs to be, before the inode is completely destroyed and put
 * on the inode free list.
 */
static void auditfs_evict_inode(struct inode *inode)
{
	struct inode *lower_inode;

	truncate_inode_pages(&inode->i_data, 0);
	end_writeback(inode);
	/*
	 * Decrement a reference to a lower_inode, which was incremented
	 * by our read_inode when it was created initially.
	 */
	lower_inode = auditfs_lower_inode(inode);
	auditfs_set_lower_inode(inode, NULL);
	iput(lower_inode);
}

static struct inode *auditfs_alloc_inode(struct super_block *sb)
{
	struct auditfs_inode_info *i;

	i = kmem_cache_alloc(auditfs_inode_cachep, GFP_KERNEL);
	if (!i)
		return NULL;

	/* memset everything up to the inode to 0 */
	memset(i, 0, offsetof(struct auditfs_inode_info, vfs_inode));

	i->vfs_inode.i_version = 1;
	return &i->vfs_inode;
}

static void auditfs_destroy_inode(struct inode *inode)
{
	kmem_cache_free(auditfs_inode_cachep, AUDITFS_I(inode));
}

/* auditfs inode cache constructor */
static void init_once(void *obj)
{
	struct auditfs_inode_info *i = obj;

	inode_init_once(&i->vfs_inode);
}

int auditfs_init_inode_cache(void)
{
	int err = 0;

	auditfs_inode_cachep =
		kmem_cache_create("auditfs_inode_cache",
				  sizeof(struct auditfs_inode_info), 0,
				  SLAB_RECLAIM_ACCOUNT, init_once);
	if (!auditfs_inode_cachep)
		err = -ENOMEM;
	return err;
}

/* auditfs inode cache destructor */
void auditfs_destroy_inode_cache(void)
{
	if (auditfs_inode_cachep)
		kmem_cache_destroy(auditfs_inode_cachep);
}

/*
 * Used only in nfs, to kill any pending RPC tasks, so that subsequent
 * code can actually succeed and won't leave tasks that need handling.
 */
static void auditfs_umount_begin(struct super_block *sb)
{
	struct super_block *lower_sb;

	lower_sb = auditfs_lower_super(sb);
	if (lower_sb && lower_sb->s_op && lower_sb->s_op->umount_begin)
		lower_sb->s_op->umount_begin(lower_sb);
}

const struct super_operations auditfs_sops = {
	.put_super	= auditfs_put_super,
	.statfs		= auditfs_statfs,
	.remount_fs	= auditfs_remount_fs,
	.evict_inode	= auditfs_evict_inode,
	.umount_begin	= auditfs_umount_begin,
	.show_options	= generic_show_options,
	.alloc_inode	= auditfs_alloc_inode,
	.destroy_inode	= auditfs_destroy_inode,
	.drop_inode	= generic_delete_inode,
};

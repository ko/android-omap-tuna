AUDITFS_VERSION="0.1"

EXTRA_CFLAGS += -DAUDITFS_VERSION=\"$(AUDITFS_VERSION)\"

obj-$(CONFIG_AUDIT_FS) += auditfs.o

auditfs-y := dentry.o file.o inode.o main.o super.o lookup.o mmap.o

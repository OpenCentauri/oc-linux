// SPDX-License-Identifier: GPL-2.0+
/*
 * KBUF - Kernel Buffer Device Driver
 *
 * Copyright (C) Timo V
 *
 * Description: kernel buffer devices
 * Version: 0.0.1
 *
 * This driver provides character device interfaces for kernel buffer
 * management with mmap support for userspace access.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#define KBUF_DEV_NAME		"kbuf"
#define KBUF_CLASS_NAME		"kbuf"
#define KBUF_VERSION		"0.0.1"

/* Buffer name size - must match userspace */
#define KBUF_NAME_SIZE		32

/* Buffer types */
#define KBUF_TYPE_NONCACHE	(1)

/* IOCTL command definitions - must match userspace header */
#define IOCTL_OPT_CREATE_MAGIC		(0x100)
#define IOCTL_OPT_DESTROY_MAGIC		(0x200)
#define IOCTL_OPT_GET_MAGIC		(0x300)
#define IOCTL_OPT_MASK			(0xFF00)
#define IOCTL_TYPE_NONCACHE_BUF_MAGIC	(0x1)
#define IOCTL_TYPE_MASK			(0xFF)

#define KBUF_MGR_DEV_IOCTL_CREATE_NONCACHE_BUF	(IOCTL_TYPE_NONCACHE_BUF_MAGIC | IOCTL_OPT_CREATE_MAGIC)
#define KBUF_MGR_DEV_IOCTL_DESTROY_NONCACHE_BUF	(IOCTL_TYPE_NONCACHE_BUF_MAGIC | IOCTL_OPT_DESTROY_MAGIC)
#define KBUF_MGR_DEV_IOCTL_GET_NONCACHE_BUF	(IOCTL_TYPE_NONCACHE_BUF_MAGIC | IOCTL_OPT_GET_MAGIC)
#define KBUF_MGR_DEV_IOCTL_CREATE_BUF		(IOCTL_OPT_CREATE_MAGIC)
#define KBUF_MGR_DEV_IOCTL_DESTROY_BUF		(IOCTL_OPT_DESTROY_MAGIC)
#define KBUF_MGR_DEV_IOCTL_GET_BUF		(IOCTL_OPT_GET_MAGIC)

/*
 * Buffer data structure - shared between kernel and userspace
 * This must match the userspace kbuf_buf_data_t exactly
 */
struct kbuf_buf_data {
	char name[KBUF_NAME_SIZE];	/* Buffer name */
	unsigned int len;		/* Buffer length */
	unsigned int type;		/* Buffer type (KBUF_TYPE_*) */
	int minor;			/* Minor device number */
	unsigned long va;		/* Kernel virtual address */
	unsigned long pa;		/* Physical address */
};

/* Forward declarations for internal structures */
struct kbuf_map_dev;
struct kbuf_mgr_dev;

/* Internal buffer structure */
struct kbuf_buffer {
	struct list_head list;
	char name[KBUF_NAME_SIZE];	/* Buffer name */
	void *virt_addr;		/* Kernel virtual address */
	dma_addr_t phys_addr;		/* Physical/DMA address */
	size_t size;			/* Buffer size */
	unsigned int type;		/* Buffer type */
	int minor;			/* Minor number of map device */
	atomic_t refcount;		/* Reference count */
	struct kbuf_map_dev *map_dev;	/* Associated map device */
};

/* Map device structure */
struct kbuf_map_dev {
	struct list_head list;
	struct cdev cdev;
	dev_t devno;
	struct device *device;
	struct kbuf_buffer *buf_data;
	char name[KBUF_NAME_SIZE];	/* Buffer name for device path */
	int minor;			/* Minor number */
	struct mutex lock;
	atomic_t open_count;
	void *priv_data;
};

/* Manager device structure */
struct kbuf_mgr_dev {
	struct list_head list;
	struct cdev cdev;
	dev_t devno;
	struct device *device;
	struct list_head buffers;
	struct list_head map_devs;
	struct mutex lock;
	spinlock_t buf_lock;
	atomic_t open_count;
	void *priv_data;
	int mgr_index;			/* Manager index (0, 1, ...) */
};

/* Global kbuf device structure */
struct kbuf_device {
	dev_t devno;			/* Device number base */
	int major;			/* Major number */
	struct class *class;		/* Device class */
	struct cdev mgr_cdev;		/* Manager char device */
	struct device *mgr_device;	/* Manager device (char dev) */
	struct device *dma_device;	/* Device for DMA operations (platform dev) */
	struct list_head mgr_devs;	/* List of manager devices */
	struct list_head map_devs;	/* List of map devices */
	struct list_head buffers;	/* List of all buffers */
	struct mutex lock;		/* Global lock */
	atomic_t minor_counter;		/* Minor number counter */
	atomic_t mgr_counter;		/* Manager index counter */
	struct platform_device *pdev;	/* Platform device */
	bool initialized;
};

static struct kbuf_device *kbuf_dev;

/* Forward declarations */
static int kbuf_map_dev_open(struct inode *inode, struct file *file);
static int kbuf_map_dev_release(struct inode *inode, struct file *file);
static int kbuf_map_dev_mmap(struct file *file, struct vm_area_struct *vma);
static long kbuf_map_dev_unlocked_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg);
static int kbuf_mgr_dev_open(struct inode *inode, struct file *file);
static int kbuf_mgr_dev_release(struct inode *inode, struct file *file);
static long kbuf_mgr_dev_unlocked_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg);

/* Map device file operations */
static const struct file_operations kbuf_map_dev_fops = {
	.owner = THIS_MODULE,
	.open = kbuf_map_dev_open,
	.release = kbuf_map_dev_release,
	.mmap = kbuf_map_dev_mmap,
	.unlocked_ioctl = kbuf_map_dev_unlocked_ioctl,
	.compat_ioctl = kbuf_map_dev_unlocked_ioctl,
};

/* Manager device file operations */
static const struct file_operations kbuf_mgr_dev_fops = {
	.owner = THIS_MODULE,
	.open = kbuf_mgr_dev_open,
	.release = kbuf_mgr_dev_release,
	.unlocked_ioctl = kbuf_mgr_dev_unlocked_ioctl,
	.compat_ioctl = kbuf_mgr_dev_unlocked_ioctl,
};

/**
 * Get the kbuf device class
 * @return pointer to device class, or NULL if not initialized
 */
struct class *kbuf_get_class(void)
{
	if (!kbuf_dev || !kbuf_dev->initialized)
		return NULL;
	return kbuf_dev->class;
}
EXPORT_SYMBOL(kbuf_get_class);

/**
 * Get the kbuf major device number
 * @return major device number, or 0 if not initialized
 */
int kbuf_get_dev_major(void)
{
	if (!kbuf_dev || !kbuf_dev->initialized)
		return 0;
	return kbuf_dev->major;
}
EXPORT_SYMBOL(kbuf_get_dev_major);

/**
 * Get a new minor device number
 * @return allocated minor number, or negative error
 */
int kbuf_get_dev_minor(void)
{
	int minor;

	if (!kbuf_dev || !kbuf_dev->initialized)
		return -ENODEV;

	minor = atomic_inc_return(&kbuf_dev->minor_counter);
	return minor;
}
EXPORT_SYMBOL(kbuf_get_dev_minor);

/**
 * Release a minor device number
 * @param minor: minor number to release
 */
void kbuf_remove_dev_minor(int minor)
{
	/* Minor numbers are tracked via atomic counter, no explicit release needed */
	(void)minor;
}
EXPORT_SYMBOL(kbuf_remove_dev_minor);

/**
 * Allocate a kernel buffer (internal)
 * @param size: size of buffer to allocate
 * @param type: buffer type (KBUF_TYPE_*)
 * @return pointer to buffer structure, or NULL on failure
 */
static struct kbuf_buffer *kbuf_alloc_buffer_internal(size_t size, unsigned int type)
{
	struct kbuf_buffer *buf;
	struct device *dev;

	if (!kbuf_dev || !kbuf_dev->initialized || !kbuf_dev->dma_device)
		return NULL;

	/* Use platform device for DMA - it has the DMA mask set */
	dev = kbuf_dev->dma_device;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	/* Allocate DMA coherent memory (non-cached) for KBUF_TYPE_NONCACHE */
	if (type == KBUF_TYPE_NONCACHE) {
		buf->virt_addr = dma_alloc_coherent(dev, size, &buf->phys_addr,
						     GFP_KERNEL | __GFP_ZERO);
	} else {
		/* Default to coherent allocation */
		buf->virt_addr = dma_alloc_coherent(dev, size, &buf->phys_addr,
						     GFP_KERNEL | __GFP_ZERO);
	}

	if (!buf->virt_addr) {
		dev_err(dev, "Failed to allocate %zu bytes\n", size);
		kfree(buf);
		return NULL;
	}

	buf->size = size;
	buf->type = type;
	atomic_set(&buf->refcount, 1);
	INIT_LIST_HEAD(&buf->list);

	/* Add to global buffer list */
	mutex_lock(&kbuf_dev->lock);
	list_add(&buf->list, &kbuf_dev->buffers);
	mutex_unlock(&kbuf_dev->lock);

	dev_dbg(dev, "Allocated buffer: virt=%pK phys=%pad size=%zu\n",
		buf->virt_addr, &buf->phys_addr, size);

	return buf;
}

/**
 * Free a kernel buffer (internal)
 * @param buf: buffer to free
 */
static void kbuf_free_buffer_internal(struct kbuf_buffer *buf)
{
	struct device *dev;

	if (!buf || !kbuf_dev || !kbuf_dev->dma_device)
		return;

	dev = kbuf_dev->dma_device;

	if (atomic_dec_and_test(&buf->refcount)) {
		mutex_lock(&kbuf_dev->lock);
		list_del(&buf->list);
		mutex_unlock(&kbuf_dev->lock);

		if (buf->virt_addr)
			dma_free_coherent(dev, buf->size, buf->virt_addr,
					   buf->phys_addr);

		dev_dbg(dev, "Freed buffer: phys=%pad size=%zu\n",
			&buf->phys_addr, buf->size);

		kfree(buf);
	}
}

/**
 * Add a map device for a buffer
 * @param	buf: buffer to create map device for
 *
 * @return pointer to map device, or NULL on failure
 */
struct kbuf_map_dev *kbuf_map_dev_add(struct kbuf_buffer *buf)
{
	struct kbuf_map_dev *map_dev;
	int minor;
	int ret;

	if (!kbuf_dev || !kbuf_dev->initialized || !buf)
		return NULL;

	map_dev = kzalloc(sizeof(*map_dev), GFP_KERNEL);
	if (!map_dev)
		return NULL;

	minor = kbuf_get_dev_minor();
	if (minor < 0) {
		kfree(map_dev);
		return NULL;
	}

	map_dev->devno = MKDEV(kbuf_dev->major, minor);
	map_dev->buf_data = buf;
	map_dev->minor = minor;
	strncpy(map_dev->name, buf->name, KBUF_NAME_SIZE - 1);
	map_dev->name[KBUF_NAME_SIZE - 1] = '\0';
	mutex_init(&map_dev->lock);
	atomic_set(&map_dev->open_count, 0);
	INIT_LIST_HEAD(&map_dev->list);

	cdev_init(&map_dev->cdev, &kbuf_map_dev_fops);
	map_dev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&map_dev->cdev, map_dev->devno, 1);
	if (ret) {
		dev_err(kbuf_dev->mgr_device, "Failed to add map cdev\n");
		kfree(map_dev);
		return NULL;
	}

	/* Create device with name format: kbuf-map-<minor>-<name> */
	map_dev->device = device_create(kbuf_dev->class, kbuf_dev->mgr_device,
					 map_dev->devno, map_dev,
					 "kbuf-map-%d-%s", minor, buf->name);
	if (IS_ERR(map_dev->device)) {
		cdev_del(&map_dev->cdev);
		kfree(map_dev);
		return NULL;
	}

	buf->map_dev = map_dev;

	mutex_lock(&kbuf_dev->lock);
	list_add(&map_dev->list, &kbuf_dev->map_devs);
	mutex_unlock(&kbuf_dev->lock);

	return map_dev;
}
EXPORT_SYMBOL(kbuf_map_dev_add);

/**
 * Find map device by buffer data
 * @param buf_data: buffer data to search for
 * @return pointer to map device, or NULL if not found
 */
struct kbuf_map_dev *kbuf_map_dev_get_by_buf_data(void *buf_data)
{
	struct kbuf_map_dev *map_dev;

	if (!kbuf_dev || !kbuf_dev->initialized)
		return NULL;

	mutex_lock(&kbuf_dev->lock);
	list_for_each_entry(map_dev, &kbuf_dev->map_devs, list) {
		if (map_dev->buf_data == buf_data) {
			mutex_unlock(&kbuf_dev->lock);
			return map_dev;
		}
	}
	mutex_unlock(&kbuf_dev->lock);

	return NULL;
}
EXPORT_SYMBOL(kbuf_map_dev_get_by_buf_data);

/**
 * Find and delete map device by buffer
 * @param buf_data: buffer data to search for
 * @return 0 on success, negative error code on failure
 */
int kbuf_map_dev_find_and_del_by_buf_data(void *buf_data)
{
	struct kbuf_map_dev *map_dev, *tmp;

	if (!kbuf_dev || !kbuf_dev->initialized)
		return -ENODEV;

	mutex_lock(&kbuf_dev->lock);
	list_for_each_entry_safe(map_dev, tmp, &kbuf_dev->map_devs, list) {
		if (map_dev->buf_data == buf_data) {
			list_del(&map_dev->list);
			mutex_unlock(&kbuf_dev->lock);

			device_destroy(kbuf_dev->class, map_dev->devno);
			cdev_del(&map_dev->cdev);
			kfree(map_dev);
			return 0;
		}
	}
	mutex_unlock(&kbuf_dev->lock);

	return -ENOENT;
}
EXPORT_SYMBOL(kbuf_map_dev_find_and_del_by_buf_data);

/**
 * Find map device by buffer data (alias)
 * @param buf_data: buffer data to search for
 *
 * @return pointer to map device, or NULL if not found
 */
struct kbuf_map_dev *kbuf_map_dev_find_by_buf_data(void *buf_data)
{
	return kbuf_map_dev_get_by_buf_data(buf_data);
}
EXPORT_SYMBOL(kbuf_map_dev_find_by_buf_data);

/**
 * kbuf_register_mgr_dev - Register a manager device
 * @priv: private data for the manager device
 *
 * Returns: pointer to manager device, or NULL on failure
 */
struct kbuf_mgr_dev *kbuf_register_mgr_dev(void *priv)
{
	struct kbuf_mgr_dev *mgr_dev;
	int minor;
	int mgr_index;
	int ret;

	if (!kbuf_dev || !kbuf_dev->initialized)
		return NULL;

	mgr_dev = kzalloc(sizeof(*mgr_dev), GFP_KERNEL);
	if (!mgr_dev)
		return NULL;

	minor = kbuf_get_dev_minor();
	if (minor < 0) {
		kfree(mgr_dev);
		return NULL;
	}

	/* Assign manager index */
	mgr_index = atomic_fetch_add(1, &kbuf_dev->mgr_counter);

	mgr_dev->devno = MKDEV(kbuf_dev->major, minor);
	mgr_dev->priv_data = priv;
	mgr_dev->mgr_index = mgr_index;
	mutex_init(&mgr_dev->lock);
	spin_lock_init(&mgr_dev->buf_lock);
	atomic_set(&mgr_dev->open_count, 0);
	INIT_LIST_HEAD(&mgr_dev->list);
	INIT_LIST_HEAD(&mgr_dev->buffers);
	INIT_LIST_HEAD(&mgr_dev->map_devs);

	cdev_init(&mgr_dev->cdev, &kbuf_mgr_dev_fops);
	mgr_dev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&mgr_dev->cdev, mgr_dev->devno, 1);
	if (ret) {
		dev_err(kbuf_dev->mgr_device, "Failed to add mgr cdev\n");
		kfree(mgr_dev);
		return NULL;
	}

	/* Create device with name format: kbuf-mgr-<index> */
	mgr_dev->device = device_create(kbuf_dev->class, kbuf_dev->mgr_device,
					 mgr_dev->devno, mgr_dev,
					 "kbuf-mgr-%d", mgr_index);
	if (IS_ERR(mgr_dev->device)) {
		cdev_del(&mgr_dev->cdev);
		kfree(mgr_dev);
		return NULL;
	}

	mutex_lock(&kbuf_dev->lock);
	list_add(&mgr_dev->list, &kbuf_dev->mgr_devs);
	mutex_unlock(&kbuf_dev->lock);

	return mgr_dev;
}
EXPORT_SYMBOL(kbuf_register_mgr_dev);

/**
 * Unregister a manager device
 * @param mgr_dev manager device to unregister
 */
void kbuf_unregister_mgr_dev(struct kbuf_mgr_dev *mgr_dev)
{
	struct kbuf_buffer *buf, *buf_tmp;

	if (!mgr_dev || !kbuf_dev)
		return;

	mutex_lock(&kbuf_dev->lock);
	list_del(&mgr_dev->list);
	mutex_unlock(&kbuf_dev->lock);

	/* Free all buffers associated with this manager */
	mutex_lock(&mgr_dev->lock);
	list_for_each_entry_safe(buf, buf_tmp, &mgr_dev->buffers, list) {
		kbuf_free_buffer_internal(buf);
	}
	mutex_unlock(&mgr_dev->lock);

	device_destroy(kbuf_dev->class, mgr_dev->devno);
	cdev_del(&mgr_dev->cdev);
	kfree(mgr_dev);
}
EXPORT_SYMBOL(kbuf_unregister_mgr_dev);

/**
 * Register a map device
 * @param buf buffer to associate with map device
 * @return pointer to map device, or NULL on failure
 */
struct kbuf_map_dev *kbuf_register_map_dev(struct kbuf_buffer *buf)
{
	return kbuf_map_dev_add(buf);
}
EXPORT_SYMBOL(kbuf_register_map_dev);

/**
 * Unregister a map device
 * @param map_dev map device to unregister
 */
void kbuf_unregister_map_dev(struct kbuf_map_dev *map_dev)
{
	if (!map_dev || !kbuf_dev)
		return;

	mutex_lock(&kbuf_dev->lock);
	list_del(&map_dev->list);
	mutex_unlock(&kbuf_dev->lock);

	device_destroy(kbuf_dev->class, map_dev->devno);
	cdev_del(&map_dev->cdev);
	kfree(map_dev);
}
EXPORT_SYMBOL(kbuf_unregister_map_dev);

/**
 * Map device open
 * @param inode inode pointer
 * @param file file pointer
 * @return 0 on success, negative error code on failure
 */
static int kbuf_map_dev_open(struct inode *inode, struct file *file)
{
	struct kbuf_map_dev *map_dev;

	map_dev = container_of(inode->i_cdev, struct kbuf_map_dev, cdev);
	if (!map_dev)
		return -ENODEV;

	file->private_data = map_dev;
	atomic_inc(&map_dev->open_count);

	dev_dbg(kbuf_dev->mgr_device, "Map device opened, count=%d\n",
		atomic_read(&map_dev->open_count));

	return 0;
}

/**
 * Map device release
 * @param inode inode pointer
 * @param file file pointer
 * @return 0 on success, negative error code on failure
 */
static int kbuf_map_dev_release(struct inode *inode, struct file *file)
{
	struct kbuf_map_dev *map_dev = file->private_data;

	if (!map_dev)
		return -ENODEV;

	atomic_dec(&map_dev->open_count);

	dev_dbg(kbuf_dev->mgr_device, "Map device released, count=%d\n",
		atomic_read(&map_dev->open_count));

	return 0;
}

/* Map device mmap - map kernel buffer to userspace */
static int kbuf_map_dev_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct kbuf_map_dev *map_dev = file->private_data;
	struct kbuf_buffer *buf;
	unsigned long size;
	int ret;

	if (!map_dev || !map_dev->buf_data)
		return -ENODEV;

	buf = map_dev->buf_data;
	size = vma->vm_end - vma->vm_start;

	if (size > buf->size) {
		dev_err(kbuf_dev->mgr_device,
			"Requested mmap size %lu exceeds buffer size %zu\n",
			size, buf->size);
		return -EINVAL;
	}

	/* Set page protection for non-cached access */
#ifdef CONFIG_KBUF_TYPE_NONCACHE
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#else
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
#endif

	ret = remap_pfn_range(vma, vma->vm_start,
			      buf->phys_addr >> PAGE_SHIFT,
			      size, vma->vm_page_prot);
	if (ret) {
		dev_err(kbuf_dev->mgr_device, "remap_pfn_range failed: %d\n", ret);
		return ret;
	}

	dev_dbg(kbuf_dev->mgr_device, "Mapped buffer phys=%pad size=%lu\n",
		&buf->phys_addr, size);

	return 0;
}

/* Map device ioctl - minimal implementation for buffer info */
static long kbuf_map_dev_unlocked_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	struct kbuf_map_dev *map_dev = file->private_data;
	struct kbuf_buffer *buf;
	struct kbuf_buf_data buf_data;

	if (!map_dev || !map_dev->buf_data)
		return -ENODEV;

	buf = map_dev->buf_data;

	switch (cmd) {
	case KBUF_MGR_DEV_IOCTL_GET_BUF:
		/* Return buffer info */
		memset(&buf_data, 0, sizeof(buf_data));
		strncpy(buf_data.name, buf->name, KBUF_NAME_SIZE - 1);
		buf_data.len = buf->size;
		buf_data.type = buf->type;
		buf_data.minor = buf->minor;
		buf_data.va = (unsigned long)buf->virt_addr;
		buf_data.pa = buf->phys_addr;
		if (copy_to_user((void __user *)arg, &buf_data, sizeof(buf_data)))
			return -EFAULT;
		break;

	default:
		return -ENOTTY;
	}

	return 0;
}

/* Manager device open */
static int kbuf_mgr_dev_open(struct inode *inode, struct file *file)
{
	struct kbuf_mgr_dev *mgr_dev;

	mgr_dev = container_of(inode->i_cdev, struct kbuf_mgr_dev, cdev);
	if (!mgr_dev)
		return -ENODEV;

	file->private_data = mgr_dev;
	atomic_inc(&mgr_dev->open_count);

	dev_dbg(kbuf_dev->mgr_device, "Manager device opened, count=%d\n",
		atomic_read(&mgr_dev->open_count));

	return 0;
}

/* Manager device release */
static int kbuf_mgr_dev_release(struct inode *inode, struct file *file)
{
	struct kbuf_mgr_dev *mgr_dev = file->private_data;

	if (!mgr_dev)
		return -ENODEV;

	atomic_dec(&mgr_dev->open_count);

	dev_dbg(kbuf_dev->mgr_device, "Manager device released, count=%d\n",
		atomic_read(&mgr_dev->open_count));

	return 0;
}

/* Manager device ioctl */
static long kbuf_mgr_dev_unlocked_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	struct kbuf_mgr_dev *mgr_dev = file->private_data;
	struct kbuf_buf_data buf_data;
	struct kbuf_buffer *buf;
	struct kbuf_map_dev *map_dev;
	int minor;

	if (!kbuf_dev || !kbuf_dev->initialized || !mgr_dev)
		return -ENODEV;

	switch (cmd) {
	case KBUF_MGR_DEV_IOCTL_CREATE_BUF:
		if (copy_from_user(&buf_data, (void __user *)arg,
				   sizeof(buf_data)))
			return -EFAULT;

		/* Allocate buffer based on type */
		buf = kbuf_alloc_buffer_internal(buf_data.len, buf_data.type);
		if (!buf)
			return -ENOMEM;

		/* Set buffer name and type */
		strncpy(buf->name, buf_data.name, KBUF_NAME_SIZE - 1);
		buf->name[KBUF_NAME_SIZE - 1] = '\0';
		buf->type = buf_data.type;

		/* Create map device for this buffer */
		map_dev = kbuf_map_dev_add(buf);
		if (!map_dev) {
			kbuf_free_buffer_internal(buf);
			return -ENOMEM;
		}

		/* Get minor number for the map device */
		minor = MINOR(map_dev->devno);
		buf->minor = minor;

		/* Fill in return values */
		buf_data.pa = buf->phys_addr;
		buf_data.va = (unsigned long)buf->virt_addr;
		buf_data.minor = minor;

		if (copy_to_user((void __user *)arg, &buf_data,
				 sizeof(buf_data))) {
			kbuf_map_dev_find_and_del_by_buf_data(buf);
			kbuf_free_buffer_internal(buf);
			return -EFAULT;
		}
		break;

	case KBUF_MGR_DEV_IOCTL_DESTROY_BUF:
		if (copy_from_user(&buf_data, (void __user *)arg,
				   sizeof(buf_data)))
			return -EFAULT;

		/* Find buffer by name and destroy it */
		mutex_lock(&kbuf_dev->lock);
		list_for_each_entry(buf, &kbuf_dev->buffers, list) {
			if (strncmp(buf->name, buf_data.name, KBUF_NAME_SIZE) == 0) {
				mutex_unlock(&kbuf_dev->lock);
				kbuf_map_dev_find_and_del_by_buf_data(buf);
				kbuf_free_buffer_internal(buf);
				return 0;
			}
		}
		mutex_unlock(&kbuf_dev->lock);
		return -ENOENT;

	case KBUF_MGR_DEV_IOCTL_GET_BUF:
		if (copy_from_user(&buf_data, (void __user *)arg,
				   sizeof(buf_data)))
			return -EFAULT;

		/* Find buffer by name and return info */
		mutex_lock(&kbuf_dev->lock);
		list_for_each_entry(buf, &kbuf_dev->buffers, list) {
			if (strncmp(buf->name, buf_data.name, KBUF_NAME_SIZE) == 0) {
				buf_data.pa = buf->phys_addr;
				buf_data.va = (unsigned long)buf->virt_addr;
				buf_data.len = buf->size;
				buf_data.type = buf->type;
				buf_data.minor = buf->minor;
				mutex_unlock(&kbuf_dev->lock);
				if (copy_to_user((void __user *)arg, &buf_data,
						 sizeof(buf_data)))
					return -EFAULT;
				return 0;
			}
		}
		mutex_unlock(&kbuf_dev->lock);
		return -ENOENT;

	default:
		return -ENOTTY;
	}

	return 0;
}

/* Platform device probe */
static int kbuf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	dev_info(dev, "KBUF driver v%s probing\n", KBUF_VERSION);

	if (kbuf_dev && kbuf_dev->initialized) {
		dev_warn(dev, "KBUF already initialized\n");
		return -EEXIST;
	}

	kbuf_dev = devm_kzalloc(dev, sizeof(*kbuf_dev), GFP_KERNEL);
	if (!kbuf_dev)
		return -ENOMEM;

	kbuf_dev->pdev = pdev;
	mutex_init(&kbuf_dev->lock);
	atomic_set(&kbuf_dev->minor_counter, 0);
	INIT_LIST_HEAD(&kbuf_dev->mgr_devs);
	INIT_LIST_HEAD(&kbuf_dev->map_devs);
	INIT_LIST_HEAD(&kbuf_dev->buffers);

	/* Allocate character device region */
	ret = alloc_chrdev_region(&kbuf_dev->devno, 0, 256, KBUF_DEV_NAME);
	if (ret < 0) {
		dev_err(dev, "Failed to allocate chrdev region: %d\n", ret);
		return ret;
	}
	kbuf_dev->major = MAJOR(kbuf_dev->devno);

	/* Create device class */
	kbuf_dev->class = class_create(THIS_MODULE, KBUF_CLASS_NAME);
	if (IS_ERR(kbuf_dev->class)) {
		ret = PTR_ERR(kbuf_dev->class);
		dev_err(dev, "Failed to create class: %d\n", ret);
		goto err_unreg_chrdev;
	}

	/* Initialize manager cdev */
	cdev_init(&kbuf_dev->mgr_cdev, &kbuf_mgr_dev_fops);
	kbuf_dev->mgr_cdev.owner = THIS_MODULE;

	ret = cdev_add(&kbuf_dev->mgr_cdev, kbuf_dev->devno, 1);
	if (ret) {
		dev_err(dev, "Failed to add cdev: %d\n", ret);
		goto err_destroy_class;
	}

	/* Create main device node: kbuf-mgr-0 */
	kbuf_dev->mgr_device = device_create(kbuf_dev->class, dev,
					      kbuf_dev->devno, kbuf_dev,
					      "kbuf-mgr-%d", 0);
	if (IS_ERR(kbuf_dev->mgr_device)) {
		ret = PTR_ERR(kbuf_dev->mgr_device);
		dev_err(dev, "Failed to create device: %d\n", ret);
		goto err_del_cdev;
	}

	/* Set DMA mask on platform device - required for DMA allocations */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "Failed to set DMA mask: %d\n", ret);
		goto err_destroy_device;
	}

	/* Store platform device for DMA operations */
	kbuf_dev->dma_device = dev;

	kbuf_dev->initialized = true;
	platform_set_drvdata(pdev, kbuf_dev);

	dev_info(dev, "KBUF driver v%s initialized, major=%d\n",
		 KBUF_VERSION, kbuf_dev->major);

	return 0;

err_destroy_device:
	device_destroy(kbuf_dev->class, kbuf_dev->devno);
err_del_cdev:
	cdev_del(&kbuf_dev->mgr_cdev);
err_destroy_class:
	class_destroy(kbuf_dev->class);
err_unreg_chrdev:
	unregister_chrdev_region(kbuf_dev->devno, 256);
	return ret;
}

/* Platform device remove */
static int kbuf_remove(struct platform_device *pdev)
{
	struct kbuf_map_dev *map_dev, *map_tmp;
	struct kbuf_mgr_dev *mgr_dev, *mgr_tmp;
	struct kbuf_buffer *buf, *buf_tmp;

	if (!kbuf_dev)
		return 0;

	kbuf_dev->initialized = false;

	/* Remove all map devices */
	mutex_lock(&kbuf_dev->lock);
	list_for_each_entry_safe(map_dev, map_tmp, &kbuf_dev->map_devs, list) {
		list_del(&map_dev->list);
		device_destroy(kbuf_dev->class, map_dev->devno);
		cdev_del(&map_dev->cdev);
		kfree(map_dev);
	}

	/* Remove all manager devices */
	list_for_each_entry_safe(mgr_dev, mgr_tmp, &kbuf_dev->mgr_devs, list) {
		list_del(&mgr_dev->list);
		device_destroy(kbuf_dev->class, mgr_dev->devno);
		cdev_del(&mgr_dev->cdev);
		kfree(mgr_dev);
	}

	/* Free all remaining buffers */
	list_for_each_entry_safe(buf, buf_tmp, &kbuf_dev->buffers, list) {
		list_del(&buf->list);
		if (buf->virt_addr)
			dma_free_coherent(&pdev->dev, buf->size, buf->virt_addr,
					   buf->phys_addr);
		kfree(buf);
	}
	mutex_unlock(&kbuf_dev->lock);

	device_destroy(kbuf_dev->class, kbuf_dev->devno);
	cdev_del(&kbuf_dev->mgr_cdev);
	class_destroy(kbuf_dev->class);
	unregister_chrdev_region(kbuf_dev->devno, 256);

	dev_info(&pdev->dev, "KBUF driver removed\n");

	return 0;
}

static const struct of_device_id kbuf_of_match[] = {
	{ .compatible = "allwinner,kbuf-driver" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, kbuf_of_match);

static struct platform_driver kbuf_driver = {
	.probe = kbuf_probe,
	.remove = kbuf_remove,
	.driver = {
		.name = KBUF_DEV_NAME,
		.of_match_table = kbuf_of_match,
	},
};

module_platform_driver(kbuf_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Timo V");
MODULE_DESCRIPTION("Kernel Buffer Devices - re-engineered from Sunxi KBUF");
MODULE_VERSION(KBUF_VERSION);

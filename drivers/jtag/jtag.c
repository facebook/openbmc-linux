// SPDX-License-Identifier: GPL-2.0
// drivers/jtag/jtag.c
//
// Copyright (c) 2017 Mellanox Technologies. All rights reserved.
// Copyright (c) 2017 Oleksandr Shamray <oleksandrs@mellanox.com>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/jtag.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/rtnetlink.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <uapi/linux/jtag.h>

#define JTAG_NAME   "jtag0"
#define MAX_JTAG_NAME_LEN (sizeof("jtag") + 5)

struct jtag {
    struct miscdevice miscdev;
    struct device *dev;
    const struct jtag_ops *ops;
    int id;
    bool opened;
    struct mutex open_lock;
    unsigned long priv[0];
};

static DEFINE_IDA(jtag_ida);

void *jtag_priv(struct jtag *jtag)
{
    return jtag->priv;
}
EXPORT_SYMBOL_GPL(jtag_priv);

static long jtag_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct jtag *jtag = file->private_data;
    struct jtag_run_test_idle idle;
    struct jtag_xfer xfer;
    struct run_cycle_param jtag_run_cycle;
    u8 *xfer_data;
    u32 data_size;
    u32 value;
    int err = 0;

    if (!arg)
        return -EINVAL;

    switch (cmd) {
    case JTAG_GIOCFREQ:
        if (!jtag->ops->freq_get)
            err = -EOPNOTSUPP;

        err = jtag->ops->freq_get(jtag, &value);
        if (err)
            break;

        if (put_user(value, (__u32 *)arg))
            err = -EFAULT;
        break;

    case JTAG_SIOCFREQ:
        if (!jtag->ops->freq_set)
            return -EOPNOTSUPP;

        if (get_user(value, (__u32 *)arg))
            return -EFAULT;
        if (value == 0)
            return -EINVAL;

        err = jtag->ops->freq_set(jtag, value);
        break;

    case JTAG_IOCRUNTEST:
        if (!jtag->ops->idle)
            return -EOPNOTSUPP;

        if (copy_from_user(&idle, (void *)arg,
                   sizeof(struct jtag_run_test_idle)))
            return -EFAULT;

        if (idle.endstate > JTAG_STATE_PAUSEDR)
            return -EINVAL;

        err = jtag->ops->idle(jtag, &idle);
        break;

    case JTAG_IOCXFER:
        if (!jtag->ops->xfer)
            return -EOPNOTSUPP;

        if (copy_from_user(&xfer, (void *)arg,
                   sizeof(struct jtag_xfer)))
            return -EFAULT;

        if (xfer.length >= JTAG_MAX_XFER_DATA_LEN)
            return -EINVAL;

        if (xfer.type > JTAG_SDR_XFER)
            return -EINVAL;

        if (xfer.direction > JTAG_WRITE_XFER)
            return -EINVAL;

        if (xfer.endstate > JTAG_STATE_PAUSEDR)
            return -EINVAL;

        data_size = DIV_ROUND_UP(xfer.length, BITS_PER_BYTE);
        xfer_data = memdup_user((void __user *)(xfer.tdio), data_size);

        if (!xfer_data)
            return -EFAULT;

        err = jtag->ops->xfer(jtag, &xfer, xfer_data);
        if (err) {
            kfree(xfer_data);
            return -EFAULT;
        }

        err = copy_to_user((void __user *)(xfer.tdio),
                   (void *)(xfer_data), data_size);

        if (err) {
            kfree(xfer_data);
            return -EFAULT;
        }

        kfree(xfer_data);
        if (copy_to_user((void *)arg, &xfer, sizeof(struct jtag_xfer)))
            return -EFAULT;
        break;

    case JTAG_GIOCSTATUS:
        if (!jtag->ops->status_get)
            return -EOPNOTSUPP;

        err = jtag->ops->status_get(jtag, &value);
        if (err)
            break;

        err = put_user(value, (__u32 *)arg);
        if (err)
            err = -EFAULT;
        break;

    case JTAG_SIOCMODE:
        if (!jtag->ops->mode_set)
            return  -EOPNOTSUPP;

        if (get_user(value, (__u32 *)arg))
            return -EFAULT;
        if (value == 0)
            return -EINVAL;

        err = jtag->ops->mode_set(jtag, value);
        break;

    case JTAG_RUN_CYCLE:
        if (copy_from_user(&jtag_run_cycle,(void __user*)arg, sizeof(struct run_cycle_param))){
          err = -EFAULT;
          break;
        }else{
          jtag->ops->run_cycle(jtag, &jtag_run_cycle);
        }

        if (copy_to_user((void __user*)(arg), &jtag_run_cycle, sizeof(struct run_cycle_param)))
          err = -EFAULT;
        break;
    default:
        return -EINVAL;
    }

    return err;
}

static int jtag_open(struct inode *inode, struct file *file)
{
    struct jtag *jtag = container_of(file->private_data, struct jtag,
                     miscdev);

    if (mutex_lock_interruptible(&jtag->open_lock))
        return -ERESTARTSYS;

    if (jtag->opened) {
        mutex_unlock(&jtag->open_lock);
        return -EBUSY;
    }
    jtag->opened = true;
    mutex_unlock(&jtag->open_lock);

    nonseekable_open(inode, file);
    file->private_data = jtag;
    return 0;
}

static int jtag_release(struct inode *inode, struct file *file)
{
    struct jtag *jtag = file->private_data;

    mutex_lock(&jtag->open_lock);
    jtag->opened = false;
    mutex_unlock(&jtag->open_lock);
    return 0;
}

static const struct file_operations jtag_fops = {
    .owner      = THIS_MODULE,
    .open       = jtag_open,
    .release    = jtag_release,
    .llseek     = noop_llseek,
    .unlocked_ioctl = jtag_ioctl,
};

struct jtag *jtag_alloc(size_t priv_size, const struct jtag_ops *ops)
{
    struct jtag *jtag;

    jtag = kzalloc(sizeof(*jtag) + priv_size, GFP_KERNEL);
    if (!jtag)
        return NULL;

    jtag->ops = ops;
    return jtag;
}
EXPORT_SYMBOL_GPL(jtag_alloc);

void jtag_free(struct jtag *jtag)
{
    kfree(jtag);
}
EXPORT_SYMBOL_GPL(jtag_free);

int jtag_register(struct jtag *jtag)
{
    char *name;
    int err;
    int id;

    id = ida_simple_get(&jtag_ida, 0, 0, GFP_KERNEL);
    if (id < 0)
        return id;

    jtag->id = id;
    jtag->opened = false;

    name = kzalloc(MAX_JTAG_NAME_LEN, GFP_KERNEL);
    if (!name) {
        err = -ENOMEM;
        goto err_jtag_alloc;
    }

    err = snprintf(name, MAX_JTAG_NAME_LEN, "jtag%d", id);
    if (err < 0)
        goto err_jtag_name;

    mutex_init(&jtag->open_lock);
    jtag->miscdev.fops =  &jtag_fops;
    jtag->miscdev.minor = MISC_DYNAMIC_MINOR;
    jtag->miscdev.name = name;

    err = misc_register(&jtag->miscdev);
    if (err) {
        dev_err(jtag->dev, "Unable to register device\n");
        goto err_jtag_name;
    }
    return 0;

err_jtag_name:
    kfree(name);
err_jtag_alloc:
    ida_simple_remove(&jtag_ida, id);
    return err;
}
EXPORT_SYMBOL_GPL(jtag_register);

void jtag_unregister(struct jtag *jtag)
{
    misc_deregister(&jtag->miscdev);
    kfree(jtag->miscdev.name);
    ida_simple_remove(&jtag_ida, jtag->id);
}
EXPORT_SYMBOL_GPL(jtag_unregister);

static void __exit jtag_exit(void)
{
    ida_destroy(&jtag_ida);
}

module_exit(jtag_exit);

MODULE_AUTHOR("Oleksandr Shamray <oleksandrs@mellanox.com>");
MODULE_DESCRIPTION("Generic jtag support");
MODULE_LICENSE("GPL v2");

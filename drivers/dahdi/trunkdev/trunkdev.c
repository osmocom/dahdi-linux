/* DAHDI Virtual Trunk device driver.
 * (C) 2022 by Harald Welte <laforge@osmocom.org>
 * for Osmocom retronetworking project
 *
 */

/*  This virtual trunk device is a new character device. Contrary to
    the per-channel devices, this is a per-span device for virtual E1
    spans.  Userspace reads and writes 32-byte E1 frames on it.

    ioctl's are added to create and delete such virtual trunk devices
    at runtime.  Each virtual trunk device is limited to one span.

    Opening trunkdev is similar to opening channels via /dev/dahd/chanw
    works:  All users open the same generic device node but then issue
    an ioctl to specify which (named) trundev they actually want to open.

    This avoids having to dynamically register minor device ids, creating
    device nodes, etc.

    The trunkdev is in RED + LOS alarm after creation.  Only once a userspace
    process opens the E1 frame side, those alarms cease.
*/


#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/wait.h>
#include <linux/spinlock.h>

#include <dahdi/kernel.h>

#include "frame_fifo.h"
#include "../dahdi.h"

#define TRUNKDEV_FMT "trunkdev/%s/0"

#define ALL_RY_ALARMS (DAHDI_ALARM_RED | DAHDI_ALARM_YELLOW | \
		       DAHDI_ALARM_LFA | DAHDI_ALARM_LMFA)

struct dahdi_trunkdev {
	/* pointer to 'file' _if_ somebody has opened us via DAHDI_TRUNKDEV_OPEM */
	struct file *file;

	/* the two E1 frame FIFOs between trunk-device and DAHDI code */
	struct frame_fifo from_trunk;
	struct frame_fifo to_trunk;

	/* wait-queue for blocking read on trunk device */
	wait_queue_head_t waitq;

	struct {
		struct dahdi_device *dev;
		struct dahdi_span span;
		struct dahdi_chan *chans[31];
		/* [next rx byte] offset into chan[i]->readchunk */
		unsigned int readchunk_idx;
		/* [next tx byte] offset into chan[i]->writechunk */
		unsigned int writechunk_idx;
	} dahdi;
};

/* process one incoming E1 frame (32 bytes; one for each TS) */
static void _span_demux_one_frame(struct dahdi_trunkdev *td, const uint8_t *data)
{
	struct dahdi_span *dspan = &td->dahdi.span;
	unsigned int i;

	/* TODO: TS0 handling */

	/* de-multiplex one byte to each channel */
	for (i = 0; i < ARRAY_SIZE(td->dahdi.chans); i++) {
		struct dahdi_chan *chan = td->dahdi.chans[i];
		chan->readchunk[td->dahdi.readchunk_idx] = data[1+i];
	}
	td->dahdi.readchunk_idx++;

	/* DAHDI_CHUNKSIZE is 8, meaning every channel (timeslot) wants data for 8
	 * bytes (PCM samples) every time _dahdi_receive() is called */
	if (td->dahdi.readchunk_idx == DAHDI_CHUNKSIZE) {
		dahdi_receive(dspan);
		td->dahdi.readchunk_idx = 0;
	}
}

/* multiplex one E1 frame (32 bytes) and write it to 'out' */
static void _span_mux_one_frame(struct dahdi_trunkdev *td, uint8_t *out)
{
	struct dahdi_span *dspan = &td->dahdi.span;
	unsigned int i;

	out[0] = 0;	/* TODO: TS0 handling */

	/* multiplex one byte of each channel into a frame */
	for (i = 0; i < ARRAY_SIZE(td->dahdi.chans); i++) {
		struct dahdi_chan *chan = td->dahdi.chans[i];
		out[1+i] = chan->writechunk[td->dahdi.writechunk_idx];
	}
	td->dahdi.writechunk_idx++;

	/* DAHDI_CHUNKSIZE is 8, meaning every channel (timeslot) has data for 8
	 * bytes (PCM samples) every time _dahdi_transmit() is called */
	if (td->dahdi.writechunk_idx == DAHDI_CHUNKSIZE) {
		dahdi_transmit(dspan);
		td->dahdi.writechunk_idx = 0;
	}
}




/*************************************************************************
 * DAHDI driver integration
 *************************************************************************/

/* configuration of the span itself */
static int td_d_spanconfig(struct file *file, struct dahdi_span *span,
			   struct dahdi_lineconfig *lc)
{
	struct dahdi_trunkdev *td = container_of(span, struct dahdi_trunkdev, dahdi.span);
	unsigned int i;

	if (lc->sync < 0)
		lc->sync = 0;
	if (lc->sync > 1) {
		// TODO: log
		lc->sync = 0;
	}

	/* (re-)set some sane defaults */
	for (i  = 0; i < span->channels; i++) {
		struct dahdi_chan *const chan = td->dahdi.chans[i];
		chan->sigcap = DAHDI_SIG_CLEAR | DAHDI_SIG_MTP2 | DAHDI_SIG_SF;
	}

	return 0;
}

/* configuration of a single channel within the span */
static int td_d_chanconfig(struct file *file, struct dahdi_chan *chan, int sigtype)
{
	return 0;
}

/* start the span */
static int td_d_startup(struct file *file, struct dahdi_span *span)
{
	struct dahdi_trunkdev *td = container_of(span, struct dahdi_trunkdev, dahdi.span);
	/* TODO: set alarms if trunkdev not open? */
	return 0;
}

/* shut down / stop the span */
static int td_d_shutdown(struct dahdi_span *span)
{
	struct dahdi_trunkdev *td = container_of(span, struct dahdi_trunkdev, dahdi.span);
	/* TODO: should trunkdev continue to read/write even if the dahdi side is shutdown */
	return 0;
}

static const char *maint2str(int cmd)
{
	switch (cmd) {
	case DAHDI_MAINT_NONE: return "NONE";
	case DAHDI_MAINT_LOCALLOOP: return "LOCALLOOP";
	case DAHDI_MAINT_NETWORKPAYLOADLOOP: return "NETWORKPAYLOADLOOP";
	default: return "UNKNOWN";
	}
}

static int td_d_maint(struct dahdi_span *span, int cmd)
{
	struct dahdi_trunkdev *td = container_of(span, struct dahdi_trunkdev, dahdi.span);

	switch (cmd) {
	case DAHDI_MAINT_NONE:
	case DAHDI_MAINT_LOCALLOOP:
	case DAHDI_MAINT_NETWORKPAYLOADLOOP:
		/* we don't need to do anything, as span->maintstat is set by the core,
		 * just acknowledge those supported modes here */
		dev_info(&td->dahdi.dev->dev, "Setting maintenance mode to: %s\n", maint2str(cmd));
		break;
	default:
		return -ENOSYS;
	}
	return 0;
}


/* called in hard_irq context with chan_lock held */
static void td_d_sync_tick(struct dahdi_span *span, int is_master)
{
	struct dahdi_trunkdev *td = container_of(span, struct dahdi_trunkdev, dahdi.span);
	unsigned int i;

	/* we must have some other clock source than this span, due to the way
	 * how DAHDI works internally: This very sync_tick function is called when the
	 * master device receives data via _dahdi_receive().  As we are calling
	 * _dahdi_receive() via _span_demux_one_frame(), this would lead to re-entrancy */
	BUG_ON(is_master);

	/* trigger transmission + reception of E1 frames */
	for (i = 0; i < DAHDI_CHUNKSIZE; i++) {
		uint8_t frame[32];
		int rc;

		switch (span->maintstat) {
		case DAHDI_MAINT_NONE:
			/* Trunk -> DAHDI direction */
			rc = frame_fifo_out(&td->from_trunk, frame);
			if (rc >= 0)
				_span_demux_one_frame(td, frame);

			/* Trunk <- DAHDI direction */
			_span_mux_one_frame(td, frame);
			frame_fifo_in(&td->to_trunk, frame);
			break;
		case DAHDI_MAINT_LOCALLOOP:
			/* Loop frame from DAHDI back to itself */
			_span_mux_one_frame(td, frame);
			_span_demux_one_frame(td, frame);
			break;
		case DAHDI_MAINT_NETWORKPAYLOADLOOP:
			/* Loop frame from trunkdev back to itself */
			rc = frame_fifo_out(&td->from_trunk, frame);
			if (rc >= 0)
				frame_fifo_in(&td->to_trunk, frame);
			break;
		}
	}
	/* wake up any processes doing blocking read or waiting in poll */
	wake_up_interruptible(&td->waitq);
}

static const struct dahdi_span_ops trunkdev_span_ops = {
	.owner = THIS_MODULE,
	.spanconfig = td_d_spanconfig,
	.chanconfig = td_d_chanconfig,
	.startup = td_d_startup,
	.shutdown = td_d_shutdown,
	.maint = td_d_maint,
	.sync_tick = td_d_sync_tick,
};


static void trunkdev_free_channels(struct dahdi_trunkdev *td)
{
	unsigned int i;
	for (i = 0; i < ARRAY_SIZE(td->dahdi.chans); i++) {
		kfree(td->dahdi.chans[i]);
		td->dahdi.chans[i] = NULL;
	}
}

static int trunkdev_alloc_channels(struct dahdi_trunkdev *td)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(td->dahdi.chans); i++) {
		struct dahdi_chan *chan;
		kfree(td->dahdi.chans[i]);
		chan = td->dahdi.chans[i] = kzalloc(sizeof(*chan), GFP_KERNEL);
		if (!chan) {
			trunkdev_free_channels(td);
			return -ENOMEM;
		}
		chan->pvt = td;
		snprintf(chan->name, sizeof(chan->name)-1, "%s/%u",
			 td->dahdi.span.name, i+1);
		chan->chanpos = i+1;
		chan->sigcap = DAHDI_SIG_CLEAR | DAHDI_SIG_MTP2 | DAHDI_SIG_SF;
	}
	return 0;
}

static int trunkdev_create(const char *name)
{
	struct dahdi_trunkdev *td;
	struct dahdi_device *ddev;
	struct dahdi_span *dspan;
	int ret = -EINVAL;
	int rc;

	td = kzalloc(sizeof(*td), GFP_KERNEL);
	if (!td)
		return -ENOMEM;

	frame_fifo_init(&td->from_trunk, 0, NULL, td);
	frame_fifo_init(&td->to_trunk, 0, NULL, td);
	init_waitqueue_head(&td->waitq);

	td->dahdi.dev = dahdi_create_device();
	if (!td->dahdi.dev) {
		printk(KERN_ERR "Error in dahdi_create_device()\b");
		goto error;
	}
	ddev = td->dahdi.dev;
	dev_set_name(&ddev->dev, "trunkdev_%s", name);
	ddev->manufacturer = "Osmocom";
	ddev->devicetype = "Virtual Trunk";
	ddev->hardware_id = "none";
	//ddev->location = FIXME;

	dspan = &td->dahdi.span;
	snprintf(dspan->name, sizeof(dspan->name), TRUNKDEV_FMT, name);
	snprintf(dspan->desc, sizeof(dspan->desc), "Virtual trunk %s span 0", name);
	dspan->channels = 31;
	dspan->spantype = SPANTYPE_DIGITAL_E1;
	dspan->deflaw = DAHDI_LAW_ALAW;
	dspan->linecompat = DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4;
	dspan->alarms = DAHDI_ALARM_RED | DAHDI_ALARM_LOS;
	dspan->chans = td->dahdi.chans;
	dspan->flags = 0;
	dspan->offset = 0;
	dspan->ops = &trunkdev_span_ops;
	dspan->cannot_provide_timing = 1;

	rc = trunkdev_alloc_channels(td);
	if (rc) {
		ret = -ENOMEM;
		goto err_free_tdev;
	}

	list_add_tail(&dspan->device_node, &ddev->spans);

	rc = dahdi_register_device(td->dahdi.dev, NULL);
	if (rc) {
		printk(KERN_ERR "Error in dahdi_register_device()\n");
		goto err_free_chans;
	}

	return dspan->spanno;

err_free_chans:
	trunkdev_free_channels(td);
err_free_tdev:
	dahdi_free_device(td->dahdi.dev);
error:
	kfree(td);
	return ret;
}

static void trunkdev_free(struct dahdi_trunkdev *td)
{
	if (td->file)
		td->file->private_data = NULL;

	/* will in turn call td_d_shutdown() */
	dahdi_unregister_device(td->dahdi.dev);
	trunkdev_free_channels(td);
	kfree(td);
}


/*************************************************************************
 * /dev/dahdi/trunkdev character device handling for a full trunk
 *************************************************************************/

/* user has opened a trunkdev device */
static int dahdi_trunkdev_open(struct inode *inode, struct file *file)
{
	const struct file_operations *original_fops;

	printk(KERN_DEBUG "%s\n", __func__);

	/* use our own file operations for this file, not the generic dahdi ones */
	original_fops = file->f_op;
	file->f_op = dahdi_trunkdev_fops;
	file->private_data = NULL;
	/* Under normal operation, this releases the reference on the DAHDI
	 * module that was created when the file was opened. dahdi_open is
	 * responsible for taking a reference out on this module before
	 * calling this function. */
	module_put(original_fops->owner);

	/* nothing else to do here, we wait for ioctls() to create new trunkdev
	 * or to associate the file handle with existing ones */
	return 0;
}

/* user has closed the trunkdev device */
static int dahdi_trunkdev_release(struct inode *inode, struct file *file)
{
	struct dahdi_trunkdev *td = file->private_data;
	struct dahdi_span *dspan;
	unsigned long flags;

	printk(KERN_DEBUG "%s\n", __func__);

	if (unlikely(!td))
		return -ENXIO;

	dspan = &td->dahdi.span;
	/* signal RED alarm once the trunkdev user is gone */
	spin_lock_irqsave(&dspan->lock, flags);
	dspan->alarms &= ~ALL_RY_ALARMS;
	dspan->alarms |= DAHDI_ALARM_RED | DAHDI_ALARM_LOS;
	dahdi_alarm_notify(dspan);
	spin_unlock_irqrestore(&dspan->lock, flags);

	/* drop the reference taken during DAHDI_TRUNKDEV_OPEN */
	put_span(dspan);

	td->file = NULL;
	file->private_data = NULL;

	return 0;
}

static long dahdi_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long data)
{
	struct dahdi_trunkdev_create create;
	struct dahdi_trunkdev_delete delete;
	struct dahdi_trunkdev_open open;
	char name[40];
	struct dahdi_trunkdev *td;
	struct dahdi_span *span;
	unsigned long flags;
	int res;

	printk(KERN_DEBUG "%s(cmd=%u)\n", __func__, cmd);

	switch (cmd) {
	case DAHDI_TRUNKDEV_CREATE: /* create an new trunkdev/span */
		printk(KERN_DEBUG "%s(cmd=DAHDI_TRUNKDEV_CREATE)\n", __func__);
		if (copy_from_user(&create, (__user const void *) data, sizeof(create)))
			return -EFAULT;
		res = trunkdev_create(create.name);
		if (res < 0)
			return res;
		create.spanno = res;
		if (copy_to_user((__user void *) data, &create, sizeof(create))) {
			trunkdev_free(td);
			return -EFAULT;
		}
		/* increase module reference count to avoid module unload until all
		 * trunkdev have been deleted again */
		if (!try_module_get(THIS_MODULE)) {
			trunkdev_free(td);
			return -ENXIO;
		}
		return 0;

	case DAHDI_TRUNKDEV_DELETE: /* destroy an existing trunkdev/span */
		printk(KERN_DEBUG "%s(cmd=DAHDI_TRUNKDEV_DELETE)\n", __func__);
		if (copy_from_user(&delete, (__user const void *) data, sizeof(delete)))
			return -EFAULT;
		snprintf(name, sizeof(name), TRUNKDEV_FMT, delete.name);
		span = dahdi_span_find_by_name_and_get(name);
		if (!span)
			return -ENODEV;
		if (span->ops != &trunkdev_span_ops) {
			/* the span with this name is not a trunkdev span */
			put_span(span);
			return -EINVAL;
		}
		td = container_of(span, struct dahdi_trunkdev, dahdi.span);
		if (td->file) {
			/* some other file already opened this span */
			put_span(span);
			return -EBUSY;
		}
		/* FIXME: some kind of mutex required? */
		/* release the reference we obtained above before free'ing underlying memory */
		put_span(span);
		trunkdev_free(td);
		/* drop the reference we hold from TRUNKDEV_CREATE time */
		module_put(THIS_MODULE);
		return 0;

	case DAHDI_TRUNKDEV_OPEN: /* open a trunkdev/span */
		printk(KERN_DEBUG "%s(cmd=DAHDI_TRUNKDEV_OPEN)\n", __func__);
		if (unlikely(file->private_data))
			return -EBUSY;
		if (copy_from_user(&open, (__user const void *) data, sizeof(open)))
			return -EFAULT;
		snprintf(name, sizeof(name), TRUNKDEV_FMT, open.name);
		span = dahdi_span_find_by_name_and_get(name);
		if (!span)
			return -ENODEV;
		if (span->ops != &trunkdev_span_ops) {
			/* the span with this name is not a trunkdev span */
			put_span(span);
			return -EINVAL;
		}
		td = container_of(span, struct dahdi_trunkdev, dahdi.span);
		if (td->file) {
			/* some other file already opened this span */
			put_span(span);
			return -EBUSY;
		}
		/* FIXME: some kind of mutex required? */
		file->private_data = td;
		td->file = file;
		/* remove all alarms */
		spin_lock_irqsave(&span->lock, flags);
		span->alarms &= ~(ALL_RY_ALARMS|DAHDI_ALARM_LOS);
		dahdi_alarm_notify(span);
		spin_unlock_irqrestore(&span->lock, flags);
		/* FIFO should be full at this point, let's flush it as now we actually
		 * have a receiver */
		frame_fifo_flush(&td->to_trunk);

		open.spanno = span->spanno;
		if (copy_to_user((__user void *) data, &open, sizeof(open)))
			return -EFAULT;
		return 0;
	default:
		printk(KERN_ERR "%s(invalid cmd=0x%x)\n", __func__, cmd);
		return -EINVAL;
	}
}

static ssize_t dahdi_trunkdev_read(struct file *file, char __user *usrbuf,
				   size_t count, loff_t *ppos)
{
	struct dahdi_trunkdev *td = file->private_data;
	size_t copied = 0;

	if (unlikely(!td))
		return -ENODEV;

	if (unlikely(count < 1))
		return -EINVAL;

	if (unlikely(count % BYTES_PER_FRAME))
		return -EINVAL;

	while (copied < count) {
		uint8_t frame[BYTES_PER_FRAME];
		int rc;

		rc = frame_fifo_out(&td->to_trunk, frame);
		if (rc < 0) {
			/* insufficient data available */
			if (file->f_flags & O_NONBLOCK) {
				if (copied == 0)
					return -EAGAIN;
				else
					return copied;
			}
			/* blocking wait for frames in FIFO or for device disappearing */
			rc = wait_event_interruptible(td->waitq,
					(frame_fifo_frames(&td->to_trunk) || !file->private_data));
			if (rc)
				return rc;
			/* device has disappeared */
			if (!file->private_data)
				return -ENODEV;
		}
		if (copy_to_user(usrbuf + copied, frame, sizeof(frame)))
			return -EFAULT;
		copied += sizeof(frame);
	}

	return copied;
}

static ssize_t dahdi_trunkdev_write(struct file *file, const char __user *usrbuf, size_t count, loff_t *ppos)
{
	struct dahdi_trunkdev *td = file->private_data;
	size_t copied = 0;

	if (unlikely(!td))
		return -ENODEV;

	if (unlikely(count < 1))
		return -EINVAL;

	if (unlikely(count % BYTES_PER_FRAME))
		return -EINVAL;

	while (copied < count) {
		uint8_t frame[BYTES_PER_FRAME];

		if (copy_from_user(frame, usrbuf + copied, sizeof(frame)))
			return -EFAULT;

		frame_fifo_in(&td->from_trunk, frame);
		copied += sizeof(frame);
		/* TODO: we could detect overflow and start blocking? */
	}

	return copied;
}

static unsigned int dahdi_trunkdev_poll(struct file *file, struct poll_table_struct *wait_table)
{
	struct dahdi_trunkdev *td = file->private_data;
	unsigned int ret = 0;

	if (unlikely(!td))
		return POLLERR | POLLHUP | POLLRDHUP | POLLNVAL | POLLPRI;

	poll_wait(file, &td->waitq, wait_table);

	if (frame_fifo_frames(&td->to_trunk))
		ret |= POLLIN | POLLRDNORM;

	/* we currently always allow write to the fifo, as we don't detect overflows yet */
	ret |= POLLOUT | POLLWRNORM;

	return ret;
}

static const struct file_operations __dahdi_trunkdev_fops = {
	.owner   = THIS_MODULE,
	.open    = dahdi_trunkdev_open,
	.release = dahdi_trunkdev_release,
	.unlocked_ioctl  = dahdi_unlocked_ioctl,
	.read    = dahdi_trunkdev_read,
	.write   = dahdi_trunkdev_write,
	.poll    = dahdi_trunkdev_poll,
};

static struct dahdi_chardev trunkdev_chardev = {
	.name = "trunkdev",
	.minor = DAHDI_TRUNKDEV,
};

static int __init dahdi_trunkdev_init(void)
{
	int res;

	if (dahdi_trunkdev_fops) {
		printk(KERN_WARNING "dahdi_trunkdev_fops already set.\n");
		return -EBUSY;
	}

	if ((res = dahdi_register_chardev(&trunkdev_chardev)))
		return res;

	dahdi_trunkdev_fops = (struct file_operations *) &__dahdi_trunkdev_fops;

	printk(KERN_INFO "%s: Loaded.\n", THIS_MODULE->name);
	return 0;
}

static void __exit dahdi_trunkdev_cleanup(void)
{
	dahdi_unregister_chardev(&trunkdev_chardev);
	dahdi_trunkdev_fops = NULL;
	printk(KERN_INFO "%s: Unloaded.\n", THIS_MODULE->name);
}

MODULE_DESCRIPTION("DAHDI virtual trunk device");
MODULE_AUTHOR("Harald Welte <laforge@osmocom.org>");
MODULE_LICENSE("GPL");

module_init(dahdi_trunkdev_init);
module_exit(dahdi_trunkdev_cleanup);

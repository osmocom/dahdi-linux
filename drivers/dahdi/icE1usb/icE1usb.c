// SPDX-License-Identifier: GPL-2.0-or-later

/* Osmocom icE1usb driver
 * Copyright (C) 2020 by Harald Welte <laforge@osmocom.org>
 * inspired by osmo-e1d by Sylvain Munaut */

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>

#include <dahdi/kernel.h>

#include "ice1usb_proto.h"

#define VERSION "0.1"
#define ICE1USB_MAX_ISOC_FRAMES 	4

/***********************************************************************
* data structures
***********************************************************************/

enum ice1usb_flags {
	ICE1USB_ISOC_RUNNING,
	ICE1USB_IRQ_RUNNING,
};

struct ice1usb {
	/* USB device and interface we operate on */
	struct usb_device *usb_dev;
	struct usb_interface *usb_intf;
	/* altsetting for 'OFF' state */
	const struct usb_host_interface *alt_off;
	/* altsetting for 'ON' state */
	const struct usb_host_interface *alt_on;
	struct {
		struct usb_anchor iso_in;
		struct usb_anchor iso_out;
		struct usb_anchor iso_fb;
		struct usb_anchor irq;
	} anchor;
	/* USB endpoint numbers */
	struct {
		const struct usb_endpoint_descriptor *iso_in;
		const struct usb_endpoint_descriptor *iso_out;
		const struct usb_endpoint_descriptor *iso_fb;
		const struct usb_endpoint_descriptor *irq;
	} ep;
	/* enum ice1usb_flags */
	unsigned long flags;
	/* feedback flow-control */
	uint32_t r_acc;
	uint32_t r_sw;
	/* DAHDI driver related bits */
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

static void ice1usb_free(struct ice1usb *ieu)
{
	usb_put_dev(ieu->usb_dev);
	kfree(ieu);
}

static void e1u_free_channels(struct ice1usb *ieu)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ieu->dahdi.chans); i++) {
		kfree(ieu->dahdi.chans[i]);
		ieu->dahdi.chans[i] = NULL;
	}
}

static int e1u_alloc_channels(struct ice1usb *ieu)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ieu->dahdi.chans); i++) {
		struct dahdi_chan *chan;
		kfree(ieu->dahdi.chans[i]);
		chan = ieu->dahdi.chans[i] = kzalloc(sizeof(*chan), GFP_KERNEL);
		if (!chan) {
			e1u_free_channels(ieu);
			return -ENOMEM;
		}

		chan->pvt = ieu;
		snprintf(chan->name, sizeof(chan->name)-1, "%s/%d",
			 ieu->dahdi.span.name, i+1);
		chan->chanpos = i+1;
		chan->sigcap = DAHDI_SIG_CLEAR | DAHDI_SIG_MTP2 | DAHDI_SIG_SF;
	}
	return 0;
}

/***********************************************************************
 * ISOCHRONOUS transfers
 ***********************************************************************/

static inline void __fill_isoc_descriptor(struct urb *urb, unsigned int max_pack_size)
{
	unsigned int i, offset = 0;

	for (i = 0; i < ICE1USB_MAX_ISOC_FRAMES; i++) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = max_pack_size;
		offset += max_pack_size;
	};
	urb->number_of_packets = i;
}

/* allocate + submit an isochronous URB for given EP; anchor it */
static int ice1usb_submit_isoc_urb(struct ice1usb *ieu,
				   const struct usb_endpoint_descriptor *ep,
				   struct usb_anchor *anchor,
				   usb_complete_t compl, gfp_t mem_flags)
{
	unsigned int max_pack_size = le16_to_cpu(ep->wMaxPacketSize);
	unsigned int pipe, size, rc;
	struct urb *urb;
	uint8_t *buf;

	urb = usb_alloc_urb(ICE1USB_MAX_ISOC_FRAMES, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = max_pack_size * ICE1USB_MAX_ISOC_FRAMES;
	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	if (ep->bEndpointAddress & USB_DIR_IN)
		pipe = usb_rcvisocpipe(ieu->usb_dev, ep->bEndpointAddress);
	else
		pipe = usb_sndisocpipe(ieu->usb_dev, ep->bEndpointAddress);

	usb_fill_int_urb(urb, ieu->usb_dev, pipe, buf, size, compl, ieu,
			 ep->bInterval);
	urb->transfer_flags = URB_FREE_BUFFER | URB_ISO_ASAP;

	__fill_isoc_descriptor(urb, max_pack_size);

	usb_anchor_urb(urb, anchor);

	rc = usb_submit_urb(urb, mem_flags);
	if (rc < 0) {
		/* -EPERM: urb is being killed; -ENODEV: device got disconnected */
		if (rc != -EPERM && rc != -ENODEV)
			dev_err(&ieu->usb_dev->dev, "urb %p submission failed (%d)", urb, -rc);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return rc;
}

/* process one incoming E1 frame (32 bytes; one for each TS) */
static void span_demux_one_frame(struct ice1usb *ieu, const uint8_t *data)
{
	struct dahdi_span *dspan = &ieu->dahdi.span;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ieu->dahdi.chans); i++) {
		struct dahdi_chan *chan = ieu->dahdi.chans[i];
		chan->readchunk[ieu->dahdi.readchunk_idx] = data[1+i];
	}
	ieu->dahdi.readchunk_idx++;

	/* DAHDI_CHUNKSIZE is 8, meaning every channel (timeslot) wants data for 8
 	 * bytes (PCM samples) every time _dahdi_receive() is called */
	if (ieu->dahdi.readchunk_idx == DAHDI_CHUNKSIZE) {
		dahdi_receive(dspan);
		ieu->dahdi.readchunk_idx = 0;
	}
}

/* IN endpoint URB completes */
static void iso_in_complete(struct urb *urb)
{
	struct ice1usb *ieu = urb->context;
	unsigned int i;
	int rc;

	//dev_dbg(&ieu->usb_dev->dev, "IN urb %p completion (%d)", urb, urb->status);

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {
			unsigned int offset = urb->iso_frame_desc[i].offset;
			unsigned int length = urb->iso_frame_desc[i].actual_length;
			unsigned int j;

			if (urb->iso_frame_desc[i].status) {
				dev_err(&ieu->usb_dev->dev, "IN urb %p frame %u status %d",
					urb, i, urb->iso_frame_desc[i].status);
				continue;
			}

			/* process received data */
			for (j = 4; j < length; j += 32)
				span_demux_one_frame(ieu, urb->transfer_buffer + offset + j);
		}
	} else if (urb->status == -ENOENT) {
		/* Avoid suspend failed when usb_kill_urb */
		return;
	}

	/* re-submit unless stopped */
	if (!test_bit(ICE1USB_ISOC_RUNNING, &ieu->flags))
		return;

	usb_anchor_urb(urb, &ieu->anchor.iso_in);
	usb_mark_last_busy(ieu->usb_dev);

	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc < 0) {
		/* -EPERM: urb is being killed; -ENODEV: device got disconnected */
		if (rc != -EPERM && rc != -ENODEV)
			dev_err(&ieu->usb_dev->dev, "urb %p submission failed (%d)", urb, -rc);
		usb_unanchor_urb(urb);
	}
}

/* Feedback endpoint URB completes */
static void iso_fb_complete(struct urb *urb)
{
	struct ice1usb *ieu = urb->context;
	unsigned int i;
	int rc;

	//dev_dbg(&ieu->usb_dev->dev, "FB urb %p completion (%d)", urb, urb->status);

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {
			unsigned int offset = urb->iso_frame_desc[i].offset;
			unsigned int length = urb->iso_frame_desc[i].actual_length;
			const uint8_t *rx = urb->transfer_buffer + offset;

			if (urb->iso_frame_desc[i].status) {
				dev_err(&ieu->usb_dev->dev, "FB urb %p frame %u status %d",
					urb, i, urb->iso_frame_desc[i].status);
			}

			if (urb->iso_frame_desc[i].status || length < 3)
				continue;

			/* process received data */
			ieu->r_sw = (rx[2] << 16) | (rx[1] << 8) | rx[0];
		}
	} else if (urb->status == -ENOENT) {
		/* Avoid suspend failed when usb_kill_urb */
		return;
	}

	/* re-submit unless stopped */
	if (!test_bit(ICE1USB_ISOC_RUNNING, &ieu->flags))
		return;

	usb_anchor_urb(urb, &ieu->anchor.iso_fb);
	usb_mark_last_busy(ieu->usb_dev);

	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc < 0) {
		/* -EPERM: urb is being killed; -ENODEV: device got disconnected */
		if (rc != -EPERM && rc != -ENODEV)
			dev_err(&ieu->usb_dev->dev, "urb %p submission failed (%d)", urb, -rc);
		usb_unanchor_urb(urb);
	}
}

/* multiplex one E1 frame (32 bytes) and write it to 'out' */
static void span_mux_one_frame(struct ice1usb *ieu, uint8_t *out)
{
	struct dahdi_span *dspan = &ieu->dahdi.span;
	unsigned int i;

	out[0] = 0;

	for (i = 0; i < ARRAY_SIZE(ieu->dahdi.chans); i++) {
		struct dahdi_chan *chan = ieu->dahdi.chans[i];
		out[1+i] = chan->writechunk[ieu->dahdi.writechunk_idx];
	}
	ieu->dahdi.writechunk_idx++;

	/* DAHDI_CHUNKSIZE is 8, meaning every channel (timeslot) wants data for 8
	 * bytes (PCM samples) every time _dahdi_receive() is called */
	if (ieu->dahdi.writechunk_idx == DAHDI_CHUNKSIZE) {
		_dahdi_transmit(dspan);
		ieu->dahdi.writechunk_idx = 0;
	}
}

/* OUT endpoint URB completes */
static void iso_out_complete(struct urb *urb)
{
	struct ice1usb *ieu = urb->context;
	unsigned int i, j;
	int rc;

	//dev_dbg(&ieu->usb_dev->dev, "OUT urb %p completion (%d) %d", urb, urb->status, urb->number_of_packets);

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {
			unsigned int offset = urb->iso_frame_desc[i].offset;
			uint8_t *tx = urb->transfer_buffer + offset;
			unsigned int fts; // frames to send

			if (urb->iso_frame_desc[i].status) {
				dev_err(&ieu->usb_dev->dev, "OUT urb %p frame %u status %d",
					urb, i, urb->iso_frame_desc[i].status);
			}

			/* flow control */
			ieu->r_acc += ieu->r_sw;
			fts = ieu->r_acc >> 10;
			if (fts < 4)
				fts = 4;
			else if (fts > 12)
				fts = 12;

			ieu->r_acc -= fts << 10;
			if (ieu->r_acc & 0x80000000)
				ieu->r_acc = 0;

			for (j = 0; j < fts; j++)
				span_mux_one_frame(ieu, tx + 4 + j*32);

			urb->iso_frame_desc[i].length = 4 + fts * 32;
		}
	} else 
		dev_err(&ieu->usb_dev->dev, "OUT urb %p completion (%d)", urb, urb->status);

	/* re-submit unless stopped */
	if (!test_bit(ICE1USB_ISOC_RUNNING, &ieu->flags))
		return;

	usb_anchor_urb(urb, &ieu->anchor.iso_out);
	usb_mark_last_busy(ieu->usb_dev);
#if 0
	dev_dbg(&ieu->usb_dev->dev, "OUT submit len=(%d,%d,%d,%d)",
		urb->iso_frame_desc[0].length, urb->iso_frame_desc[1].length,
		urb->iso_frame_desc[2].length, urb->iso_frame_desc[3].length);
#endif
	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc < 0) {
		/* -EPERM: urb is being killed; -ENODEV: device got disconnected */
		if (rc != -EPERM && rc != -ENODEV)
			dev_err(&ieu->usb_dev->dev, "urb %p submission failed (%d)", urb, -rc);
		usb_unanchor_urb(urb);
	}
}

/***********************************************************************
 * INTERRUPT transfers
 ***********************************************************************/

/* interrupt EP completes: Process and resubmit */
static void ice1usb_irq_complete(struct urb *urb)
{
	const struct ice1usb_irq *irq;
	struct ice1usb *ieu = urb->context;
	int rc;

	dev_dbg(&ieu->usb_dev->dev, "IRQ urb %p completion (%d)", urb, urb->status);

	if (urb->status == 0 && urb->actual_length >= sizeof(*irq)) {
		const struct ice1usb_irq_err *err;
		irq = (struct ice1usb_irq *) urb->transfer_buffer;
		switch (irq->type) {
		case ICE1USB_IRQQ_T_ERRCNT:
			err = &irq->u.errors;
			dev_dbg(&ieu->usb_dev->dev, "IRQ: crc=%u, align=%u, ovfl=%u, unfl=%u, flags=%x",
				le16_to_cpu(err->crc), le16_to_cpu(err->align),
				le16_to_cpu(err->ovfl), le16_to_cpu(err->unfl), err->flags);
			break;
		}
	} else if (urb->status == -ENOENT) {
		/* Avoid suspend failed when usb_kill_urb */
		return;
	}

	/* re-submit unless stopped */
	if (!test_bit(ICE1USB_IRQ_RUNNING, &ieu->flags))
		return;

	usb_anchor_urb(urb, &ieu->anchor.irq);
	usb_mark_last_busy(ieu->usb_dev);

	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc < 0) {
		/* -EPERM: urb is being killed; -ENODEV: device got disconnected */
		if (rc != -EPERM && rc != -ENODEV)
			dev_err(&ieu->usb_dev->dev, "urb %p submission failed (%d)", urb, -rc);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);
}

/* allocate + submit + anchor an URB for the interrupt endpoint */
static int ice1usb_submit_irq_urb(struct ice1usb *ieu, gfp_t mem_flags)
{
	unsigned int pipe, size, rc;
	struct urb *urb;
	uint8_t *buf;

	if (!ieu->ep.irq)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(ieu->ep.irq->wMaxPacketSize);
	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvintpipe(ieu->usb_dev, ieu->ep.irq->bEndpointAddress);

	usb_fill_int_urb(urb, ieu->usb_dev, pipe, buf, size,
			 ice1usb_irq_complete, ieu, ieu->ep.irq->bInterval);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &ieu->anchor.irq);

	rc = usb_submit_urb(urb, mem_flags);
	if (rc < 0) {
		/* -EPERM: urb is being killed; -ENODEV: device got disconnected */
		if (rc != -EPERM && rc != -ENODEV)
			dev_err(&ieu->usb_dev->dev, "urb %p submission failed (%d)", urb, -rc);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return rc;
}


/***********************************************************************
 * DAHDI integration
 ***********************************************************************/

static int e1u_d_startup(struct file *file, struct dahdi_span *span);

static int e1u_d_spanconfig(struct file *file, struct dahdi_span *span,
			    struct dahdi_lineconfig *lc)
{
	struct ice1usb *ieu = container_of(span, struct ice1usb, dahdi.span);
	unsigned int i;

	if (lc->sync < 0)
		lc->sync = 0;
	if (lc->sync > 1) {
		dev_warn(&ieu->usb_dev->dev, "Cannot set clock priority "
			"on span %d to %d\n", span->spanno, lc->sync);
		lc->sync = 0;
	}

	/* FIXME: use lc->sync to decide LOCAL/REMOTE timing source */

	for (i = 0; i < span->channels; i++) {
		struct dahdi_chan *const chan = ieu->dahdi.chans[i];
		chan->sigcap = DAHDI_SIG_CLEAR | DAHDI_SIG_MTP2 | DAHDI_SIG_SF;
	}

	/* If we're already running, then go ahead and apply the changes */
	if (span->flags & DAHDI_FLAG_RUNNING)
		e1u_d_startup(file, span);

	return 0;
}

static int e1u_d_chanconfig(struct file *file, struct dahdi_chan *chan,
			    int sigtype)
{
	struct ice1usb *ieu = chan->pvt;
	bool already_running;

	already_running = ieu->dahdi.span.flags & DAHDI_FLAG_RUNNING;
	dev_notice(&ieu->usb_dev->dev, "%sconfigured channel %d (%s) "
		   "sigtype %d\n", already_running ? "Re":"",
		   chan->channo, chan->name, sigtype);

	/* FIXME: we can probably remove this completely? */
	return 0;
}

static int ice1usb_set_altif(struct ice1usb *ieu, bool on);

static int e1u_d_startup(struct file *file, struct dahdi_span *span)
{
	struct ice1usb *ieu = container_of(span, struct ice1usb, dahdi.span);
	int rc;

	/* TODO: handle CRC4 vs. non-CRC4 case */
	//if (span->lineconfig & DAHDI_CONFIG_CRC4)

	/* Ensure we are in the right altsetting */
	rc = ice1usb_set_altif(ieu, true);
	if (rc < 0) {
		/* cancel transfers */
		return rc;
	}

	if (!test_and_set_bit(ICE1USB_ISOC_RUNNING, &ieu->flags)) {
		int i;
		for (i = 0; i < 4; i++) {
		ice1usb_submit_isoc_urb(ieu, ieu->ep.iso_in, &ieu->anchor.iso_in,
					iso_in_complete, GFP_KERNEL);
		ice1usb_submit_isoc_urb(ieu, ieu->ep.iso_fb, &ieu->anchor.iso_fb,
					iso_fb_complete, GFP_KERNEL);
		ice1usb_submit_isoc_urb(ieu, ieu->ep.iso_out, &ieu->anchor.iso_out,
					iso_out_complete, GFP_KERNEL);
		/* FIXME: clear_bit(ICE1USB_ISOC_RUNNING) on error) */
		}
	}

	if (!test_and_set_bit(ICE1USB_IRQ_RUNNING, &ieu->flags)) {
		rc = ice1usb_submit_irq_urb(ieu, GFP_KERNEL);
		if (rc) {
			dev_err(&ieu->usb_dev->dev, "error submitting IRQ ep (%d)", rc);
			clear_bit(ICE1USB_IRQ_RUNNING, &ieu->flags);
			return rc;
		}
	}

	return 0;
}

static int e1u_d_shutdown(struct dahdi_span *span)
{
	struct ice1usb *ieu = container_of(span, struct ice1usb, dahdi.span);
	int rc;

	clear_bit(ICE1USB_ISOC_RUNNING, &ieu->flags);
	clear_bit(ICE1USB_IRQ_RUNNING, &ieu->flags);

	usb_kill_anchored_urbs(&ieu->anchor.iso_in);
	usb_kill_anchored_urbs(&ieu->anchor.iso_out);
	usb_kill_anchored_urbs(&ieu->anchor.iso_fb);
	usb_kill_anchored_urbs(&ieu->anchor.irq);

	/* switch to 'off' altsetting */
	rc = ice1usb_set_altif(ieu, false);
	if (rc < 0)
		return rc;

	return 0;
}

/* set some maintenance mode according to 'cmd' */
static int e1u_d_maint(struct dahdi_span *span, int cmd)
{
	struct ice1usb *ieu = container_of(span, struct ice1usb, dahdi.span);

	switch (cmd) {
	case DAHDI_MAINT_NONE:
		dev_info(&ieu->usb_dev->dev, "Clearing all maint modes\n");
		/* FIXME */
		break;
	case DAHDI_MAINT_NETWORKPAYLOADLOOP:
		dev_info(&ieu->usb_dev->dev, "Turning on network loopback\n");
		/* FIXME */
		break;
	/* TODO: DAHDI_MAINT_*_DEFECT */
	default:
		dev_info(&ieu->usb_dev->dev, "Unknown E1 maint command: %d\n", cmd);
		return -ENOSYS;
	}

	return 0;
}

static const struct dahdi_span_ops ice1usb_span_ops = {
	.owner = THIS_MODULE,
	.spanconfig = e1u_d_spanconfig,
	.chanconfig = e1u_d_chanconfig,
	.startup = e1u_d_startup,
	.shutdown = e1u_d_shutdown,
	.maint = e1u_d_maint,
};

/***********************************************************************
 * kernel USB integration / probing 
 ***********************************************************************/

/* does the given altsetting contain an EP with wMaxPacketSize == 0 ? */
static bool has_ep_packetsize_zero(const struct usb_host_interface *alt)
{
	unsigned int j;

	/* compute the sum of all wMaxPacketSize */
	for (j = 0; j < alt->desc.bNumEndpoints; j++) {
		const struct usb_endpoint_descriptor *epd = &alt->endpoint[j].desc;
		if (usb_endpoint_xfer_isoc(epd)) {
			if (epd->wMaxPacketSize == 0)
				return true;
		}
	}
	return false;
}

/* find the 'on' altsetting (all ISOC EP wMaxPacketSize != 0) */
static const struct usb_host_interface *find_altsetting_on(struct usb_interface *intf)
{
	unsigned int i;

	for (i = 0; i < intf->num_altsetting; i++) {
		const struct usb_host_interface *alt = &intf->altsetting[i];
		if (!has_ep_packetsize_zero(alt))
			return alt;
	}
	return NULL;
}

/* find the 'off' altsetting (all ISOC EP wMaxPacketSize == 0) */
static const struct usb_host_interface *find_altsetting_off(struct usb_interface *intf)
{
	unsigned int i;

	for (i = 0; i < intf->num_altsetting; i++) {
		const struct usb_host_interface *alt = &intf->altsetting[i];
		unsigned int iso_ep_size = 0;
		unsigned int j;

		/* compute the sum of all wMaxPacketSize */
		for (j = 0; j < alt->desc.bNumEndpoints; j++) {
			const struct usb_endpoint_descriptor *epd = &alt->endpoint[j].desc;
			if (usb_endpoint_xfer_isoc(epd))
				iso_ep_size += le16_to_cpu(epd->wMaxPacketSize);
		}

		if (iso_ep_size == 0)
			return alt;
	}
	return NULL;
}

/* find the endpoint numbers within the interface */
static int find_endpoints(struct ice1usb *ieu, const struct usb_host_interface *alt)
{
	unsigned int i;
	int found = 0;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		const struct usb_endpoint_descriptor *epd = &alt->endpoint[i].desc;
		switch (usb_endpoint_type(epd)) {
		case USB_ENDPOINT_XFER_INT:
			if (!ieu->ep.irq && usb_endpoint_dir_in(epd)) {
				ieu->ep.irq = epd;
				found++;
			}
			break;
		case USB_ENDPOINT_XFER_ISOC:
			if (usb_endpoint_dir_in(epd)) {
				if ((epd->bmAttributes & USB_ENDPOINT_USAGE_MASK) ==
				    USB_ENDPOINT_USAGE_FEEDBACK) {
					if (!ieu->ep.iso_fb) {
						ieu->ep.iso_fb = epd;
						found++;
					}
				} else {
					if (!ieu->ep.iso_in) {
						ieu->ep.iso_in = epd;
						found++;
					}
				}
			} else {
				if (!ieu->ep.iso_out) {
					ieu->ep.iso_out = epd;
					found++;
				}
			}
			break;
		default:
			break;
		}
	}
	return found;
}


/* select interface altsetting on / off */
static int ice1usb_set_altif(struct ice1usb *ieu, bool on)
{
	const struct usb_interface_descriptor *desc;

	if (on)
		desc = &ieu->alt_on->desc;
	else
		desc = &ieu->alt_off->desc;

	return usb_set_interface(ieu->usb_dev, desc->bInterfaceNumber, desc->bAlternateSetting);
}


static int ice1usb_probe(struct usb_interface *intf, const struct usb_device_id *prod)
{
	struct usb_device *usb_dev = usb_get_dev(interface_to_usbdev(intf));
	const struct usb_interface_descriptor *ifdesc = &intf->altsetting->desc;
	struct dahdi_device *ddev;
	struct dahdi_span *dspan;
	struct ice1usb *ieu;
	int ret = -ENODEV;
	int rc;

	if (ifdesc->bInterfaceClass != 0xff || ifdesc->bInterfaceSubClass != 0xE1) {
		dev_err(&intf->dev, "Unsupported Interface Class/SubClass %02x/%02x",
			ifdesc->bInterfaceClass, ifdesc->bInterfaceSubClass);
		return -EINVAL;
	}

	ieu = kzalloc(sizeof(*ieu), GFP_KERNEL);
	if (!ieu) {
		dev_err(&intf->dev, "Out of memory\n");
		return -ENOMEM;
	}

	ieu->usb_dev = usb_dev;
	ieu->usb_intf = intf;
	ieu->r_acc = 0;
	ieu->r_sw = 8192;

	/* locate ON / OFF altsettings */
	ieu->alt_off = find_altsetting_off(ieu->usb_intf);
	if (!ieu->alt_off) {
		dev_err(&intf->dev, "Cannot find OFF altsetting");
		goto error;
	}
	ieu->alt_on = find_altsetting_on(ieu->usb_intf);
	if (!ieu->alt_on) {
		dev_err(&intf->dev, "Cannot find ON altsetting");
		goto error;
	}

	/* locate ON / OFF altsettings */
	if (find_endpoints(ieu, ieu->alt_on) < 4) {
		dev_err(&intf->dev, "Cannot find all endpoints");
		goto error;
	}

	/* TODO: only one dahdi_device even for multiple USB interfaces? */
	ddev = ieu->dahdi.dev = dahdi_create_device();
	if (!ddev)
		goto error;
	ddev->manufacturer = usb_dev->manufacturer;
	ddev->devicetype = usb_dev->product;
	ddev->hardware_id = usb_dev->serial;
	//ddev->location = USB_BUS_PATH;

	dspan = &ieu->dahdi.span;
	snprintf(dspan->name, sizeof(dspan->name), "icE1usb/1/%d",
		 ieu->alt_on->desc.bInterfaceNumber); //TDOO: device != 1
	snprintf(dspan->desc, sizeof(dspan->desc), "Osmocom icE1USB Card 1 Span %d",
		 ieu->alt_on->desc.bInterfaceNumber); //TODO: device != 1
	dspan->channels = 31;
	dspan->spantype = SPANTYPE_DIGITAL_E1;
	dspan->deflaw = DAHDI_LAW_ALAW;
	dspan->linecompat = DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4;
	dspan->chans = ieu->dahdi.chans;
	dspan->flags = 0;
	dspan->offset = 0;
	dspan->ops = &ice1usb_span_ops;

	rc = e1u_alloc_channels(ieu);
	if (rc) {
		ret = -ENOMEM;
		goto err_free_ddev;
	}

	list_add_tail(&dspan->device_node, &ddev->spans);

	rc = dahdi_register_device(ieu->dahdi.dev, &intf->dev);
	if (rc) {
		dev_err(&intf->dev, "Failed to reigster with DAHDI\n");
		goto err_free_chans;
	}

	init_usb_anchor(&ieu->anchor.iso_in);
	init_usb_anchor(&ieu->anchor.iso_out);
	init_usb_anchor(&ieu->anchor.iso_fb);
	init_usb_anchor(&ieu->anchor.irq);

	usb_set_intfdata(intf, ieu);

	return 0;

err_free_chans:
	e1u_free_channels(ieu);
err_free_ddev:
	dahdi_free_device(ieu->dahdi.dev);
error:
	kfree(ieu);
	return ret;
}

static void ice1usb_disconnect(struct usb_interface *intf)
{
	struct ice1usb *ieu = usb_get_intfdata(intf);

	/* FIXME: stop transfers? */

	dahdi_unregister_device(ieu->dahdi.dev);

	usb_set_intfdata(intf, NULL);
	ice1usb_free(ieu);

	dev_dbg(&intf->dev, "disconnected\n");
}

#if 0
static int ice1usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct ice1usb *ieu = usb_get_intfdata(intf);
	/* FIXME */
}

static int ice1usb_resume(struct usb_interface *intf)
{
	struct ice1usb *ieu = usb_get_intfdata(intf);
	/* FIXME */
}
#endif

static const struct usb_device_id ice1usb_products[] = {
	{
		USB_DEVICE(0x1d50, 0x6145),
	},
	{}
};

static struct usb_driver ice1usb_driver = {
	.name = "icE1usb",
	.id_table = ice1usb_products,
	.probe = ice1usb_probe,
	.disconnect = ice1usb_disconnect,
	//.suspend = ice1usb_suspend,
	//.resume = ice1usb_resume,
};

module_usb_driver(ice1usb_driver);

MODULE_AUTHOR("Harald Welte <laforge@osmocom.org>");
MODULE_DESCRIPTION("Osmocom icE1usb USB E1 interface");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);

#pragma once

#include <linux/spinlock.h>

#define BYTES_PER_FRAME		32
#define FRAMES_PER_FIFO		800	/* 100ms @ 8000 frames per second */

struct frame_fifo {
	spinlock_t lock;
	uint8_t *next_in;	/* where to write next input into FIFO */
	uint8_t *next_out;	/* where to read next output from FIFO */
	uint8_t threshold;	/* number of frames that should trigger */
	void (*threshold_cb)(struct frame_fifo *fifo, unsigned int frames, void *priv);
	void *priv;

	uint8_t buf[BYTES_PER_FRAME * FRAMES_PER_FIFO];
};

void frame_fifo_init(struct frame_fifo *fifo, unsigned int threshold,
		     void (*threshold_cb)(struct frame_fifo *fifo, unsigned int frames, void *priv),
		     void *priv);

void frame_fifo_flush(struct frame_fifo *fifo);

/* number of frames currently available in FIFO */
static inline unsigned int __frame_fifo_frames(struct frame_fifo *fifo)
{
	return ((fifo->next_in + sizeof(fifo->buf) - fifo->next_out) % sizeof(fifo->buf)) / BYTES_PER_FRAME;
}
static inline unsigned int frame_fifo_frames(struct frame_fifo *fifo)
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&fifo->lock, flags);
	ret = __frame_fifo_frames(fifo);
	spin_unlock_irqrestore(&fifo->lock, flags);

	return ret;
}

/* for how many frames do we have space in the FIFO? */
static inline unsigned int frame_fifo_space(struct frame_fifo *fifo)
{
	return FRAMES_PER_FIFO - frame_fifo_frames(fifo);
}

/* put a received frame into the FIFO */
int frame_fifo_in(struct frame_fifo *fifo, const uint8_t *frame);

/* put multiple received frames into the FIFO */
int frame_fifo_in_multi(struct frame_fifo *fifo, const uint8_t *frame, size_t count);

/* pull one frame out of the FIFO */
int frame_fifo_out(struct frame_fifo *fifo, uint8_t *out);

/* pull multiple frames out of the FIFO */
int frame_fifo_out_multi(struct frame_fifo *fifo, uint8_t *out, size_t count);

/* dump FIFO state to kernel log */
void fifo_dump(struct frame_fifo *fifo);

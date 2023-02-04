/* SPDX-License-Identifier: GPL-2.0 */

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>

typedef int16_t strength_t;

struct v4l2_heatmap {
	struct device *parent_dev;
	/* Can be NULL. Used to get the input device name */
	struct input_dev *input_dev;
	struct v4l2_device device;
	struct v4l2_pix_format format;
	struct video_device vdev;
	struct vb2_queue queue;
	struct mutex lock;
	unsigned int input_index;

	size_t width;
	size_t height;

	/* The 'frame' storage is managed by the heatmap module.
	 * The data inside frame is written inside read_frame(..)
	 * and is read inside heatmap_read(..).
	 */
	strength_t *frame;

	struct v4l2_fract timeperframe;

	/* Used to protect access to the buffer queue */
	spinlock_t heatmap_lock;
	/* guarded by heatmap_lock */
	struct list_head heatmap_buffer_list;

	/*
	 * Function read_frame must be provided by the driver.
	 * This function should write the video frame into the
	 * 'frame' field of struct v4l2_heatmap.
	 * It should return 'true' on successful heatmap read
	 * and 'false' on failure.
	 */
	bool (*read_frame)(struct v4l2_heatmap *v4l2);
};

int heatmap_probe(struct v4l2_heatmap *v4l2);
void heatmap_remove(struct v4l2_heatmap *v4l2);
/**
 * Read the heatmap and populate an available buffer with data.
 * The timestamp provided to this function will be used as the frame time.
 * Designed to be called from interrupt context.
 * This function should be called from the driver. Internally, it will call
 * read_frame(..) provided by the driver to read the actual data.
 */
void heatmap_read(struct v4l2_heatmap *v4l2, uint64_t timestamp);
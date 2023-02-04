// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/heatmap.h>
#include <media/videobuf2-vmalloc.h>

static const int FIRST_FREE_NODE = -1;

/**
 * Optimization: keep track of how many consecutive frames
 * have been dropped due to not having any buffers available.
 * If too many have been recently dropped, and still no free buffers
 * are available, then skip the bus read.
 * This situation could happen if an app have opened the video device,
 * but went into paused state, and did not close the video device in
 * onPause. With this optimization, we would avoid the wasteful bus reads
 * when no one is likely to consume the buffers.
 */
static const unsigned int NUM_BUFFERS_BEFORE_DROP = 3;
static unsigned int consecutive_frames_dropped;

struct heatmap_vb2_buffer {
	struct vb2_v4l2_buffer v4l2_vb;
	struct list_head list;
};

static int heatmap_set_input(
		struct v4l2_heatmap *v4l2, unsigned int input_index)
{
	struct v4l2_pix_format *fmt = &v4l2->format;

	if (input_index != 0)
		return -EINVAL;

	/*
	 * Changing the input implies a format change, which is not allowed
	 * while buffers for use with streaming have already been allocated.
	 */
	if (vb2_is_busy(&v4l2->queue))
		return -EBUSY;

	v4l2->input_index = input_index;

	fmt->width = v4l2->width;
	fmt->height = v4l2->height;
	fmt->pixelformat = V4L2_TCH_FMT_DELTA_TD16;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->bytesperline = fmt->width * sizeof(strength_t);
	fmt->sizeimage = fmt->width * fmt->height * sizeof(strength_t);

	return 0;
}

static inline struct heatmap_vb2_buffer *to_heatmap_vb2_buffer(
		struct vb2_buffer *vb2)
{
	return container_of(to_vb2_v4l2_buffer(vb2), struct heatmap_vb2_buffer,
			    v4l2_vb);
}

static const struct v4l2_file_operations heatmap_video_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

void heatmap_read(struct v4l2_heatmap *v4l2, uint64_t timestamp)
{
	struct heatmap_vb2_buffer *new_buf;
	struct vb2_buffer *vb2_buf;
	strength_t *data;
	int total_bytes = v4l2->format.sizeimage;
	bool read_success;

	if (!vb2_is_streaming(&v4l2->queue)) {
		/* No need to read, no one is viewing the video */
		return;
	}

	/* Optimization */
	if (consecutive_frames_dropped >= NUM_BUFFERS_BEFORE_DROP) {
		spin_lock(&v4l2->heatmap_lock);
		if (list_empty(&v4l2->heatmap_buffer_list)) {
			/*
			 * Already dropped some frames, and still don't have
			 * any free buffers. A buffer could become available
			 * during read_frame(..), but given that we already
			 * dropped some frames, this is unlikely.
			 */
			spin_unlock(&v4l2->heatmap_lock);
			return; /* Drop the frame */
		}
		spin_unlock(&v4l2->heatmap_lock);
	}

	/* This is a potentially slow operation */
	read_success = v4l2->read_frame(v4l2);
	if (!read_success)
		return;

	/* Copy the data into the buffer */
	spin_lock(&v4l2->heatmap_lock);
	if (list_empty(&v4l2->heatmap_buffer_list)) {
		/*
		 * If streaming is off, then there would
		 * be no queued buffers. This is expected.
		 * On the other hand, if there is a consumer, but there
		 * aren't any available buffers, then this indicates
		 * slowness in the userspace for reading or
		 * processing buffers.
		 */
		dev_warn(v4l2->parent_dev, "heatmap: No buffers available, dropping frame\n");
		consecutive_frames_dropped++;
		spin_unlock(&v4l2->heatmap_lock);
		return;
	}
	consecutive_frames_dropped = 0;
	new_buf = list_entry(v4l2->heatmap_buffer_list.next,
		struct heatmap_vb2_buffer, list);
	list_del(&new_buf->list);

	vb2_buf = &new_buf->v4l2_vb.vb2_buf;
	data = vb2_plane_vaddr(vb2_buf, 0);
	if (!data) {
		dev_err(v4l2->parent_dev, "heatmap: Error acquiring frame pointer\n");
		vb2_buffer_done(vb2_buf, VB2_BUF_STATE_ERROR);
		spin_unlock(&v4l2->heatmap_lock);
		return;
	}

	memcpy(data, v4l2->frame, total_bytes);
	vb2_set_plane_payload(vb2_buf, /* plane number */ 0, total_bytes);
	vb2_buf->timestamp = timestamp;
	vb2_buffer_done(vb2_buf, VB2_BUF_STATE_DONE);
	spin_unlock(&v4l2->heatmap_lock);
}
EXPORT_SYMBOL(heatmap_read);

static void heatmap_buffer_queue(struct vb2_buffer *vb)
{
	struct v4l2_heatmap *v4l2 = vb2_get_drv_priv(vb->vb2_queue);
	struct heatmap_vb2_buffer *heatmap_buffer = to_heatmap_vb2_buffer(vb);

	spin_lock(&v4l2->heatmap_lock);
	list_add_tail(&heatmap_buffer->list, &v4l2->heatmap_buffer_list);
	spin_unlock(&v4l2->heatmap_lock);
}

static int heatmap_queue_setup(struct vb2_queue *vq,
			unsigned int *num_buffers, unsigned int *num_planes,
			unsigned int sizes[], struct device *alloc_devs[])
{
	struct v4l2_heatmap *v4l2 = vb2_get_drv_priv(vq);
	size_t size = v4l2->format.sizeimage;

	if (*num_planes != 0)
		return sizes[0] < size ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = size;

	return 0;
}

static void return_all_buffers(struct v4l2_heatmap *v4l2,
	enum vb2_buffer_state state)
{
	struct heatmap_vb2_buffer *buf, *node;

	spin_lock(&v4l2->heatmap_lock);
	list_for_each_entry_safe(buf, node, &v4l2->heatmap_buffer_list, list) {
		vb2_buffer_done(&buf->v4l2_vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock(&v4l2->heatmap_lock);
}

/*
 * Stop the DMA engine. Any remaining buffers in the DMA queue are dequeued
 * and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void stop_streaming(struct vb2_queue *vq)
{
	struct v4l2_heatmap *v4l2 = vb2_get_drv_priv(vq);
	/* Release all active buffers */
	return_all_buffers(v4l2, VB2_BUF_STATE_ERROR);
}

/* V4L2 structures */
static const struct vb2_ops heatmap_queue_ops = {
	.queue_setup        = heatmap_queue_setup,
	.buf_queue          = heatmap_buffer_queue,
	.stop_streaming     = stop_streaming,
	.wait_prepare       = vb2_ops_wait_prepare,
	.wait_finish        = vb2_ops_wait_finish,
};

static const struct vb2_queue heatmap_queue = {
	.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ,
	.buf_struct_size = sizeof(struct heatmap_vb2_buffer),
	.ops = &heatmap_queue_ops,
	.mem_ops = &vb2_vmalloc_memops,
	.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
	.min_buffers_needed = 1,
};

static int heatmap_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
	struct v4l2_heatmap *v4l2 = video_drvdata(file);
	strlcpy(cap->driver, v4l2->parent_dev->driver->name,
		sizeof(cap->driver));
	if (v4l2->input_dev != NULL) {
		strlcpy(cap->card, v4l2->input_dev->name, sizeof(cap->card));
	} else {
		strlcpy(cap->card, KBUILD_MODNAME, sizeof(cap->card));
	}

	strlcpy(cap->bus_info, dev_name(v4l2->parent_dev),
		sizeof(cap->bus_info));
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_TOUCH |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int heatmap_vidioc_enum_input(struct file *file, void *priv,
		struct v4l2_input *video_input)
{
	if (video_input->index != 0)
		return -EINVAL;

	video_input->type = V4L2_INPUT_TYPE_TOUCH;
	strlcpy(video_input->name, "strength", sizeof(video_input->name));
	return 0;
}

static int heatmap_vidioc_s_input(
		struct file *file, void *priv, unsigned int input_index)
{
	struct v4l2_heatmap *v4l2 = video_drvdata(file);
	return heatmap_set_input(v4l2, input_index);
}

static int heatmap_vidioc_g_input(struct file *file, void *priv,
		unsigned int *input_index)
{
	struct v4l2_heatmap *v4l2 = video_drvdata(file);
	*input_index = v4l2->input_index;
	return 0;
}

static int heatmap_vidioc_fmt(struct file *file, void *priv,
		struct v4l2_format *fmt)
{
	struct v4l2_heatmap *v4l2 = video_drvdata(file);

	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt->fmt.pix = v4l2->format;

	return 0;
}

static int heatmap_vidioc_enum_fmt(struct file *file, void *priv,
		struct v4l2_fmtdesc *fmt)
{
	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (fmt->index != 0)
		return -EINVAL;

	fmt->pixelformat = V4L2_TCH_FMT_DELTA_TD16;
	return 0;
}

static int heatmap_vidioc_g_parm(struct file *file, void *fh,
		struct v4l2_streamparm *streamparm)
{
	struct v4l2_heatmap *v4l2 = video_drvdata(file);

	if (streamparm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	streamparm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	streamparm->parm.capture.readbuffers = 1;
	streamparm->parm.capture.timeperframe.numerator =
		v4l2->timeperframe.numerator;
	streamparm->parm.capture.timeperframe.denominator =
		v4l2->timeperframe.denominator;
	return 0;
}

static const struct v4l2_ioctl_ops heatmap_video_ioctl_ops = {
	.vidioc_querycap         = heatmap_vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = heatmap_vidioc_enum_fmt,
	.vidioc_s_fmt_vid_cap    = heatmap_vidioc_fmt,
	.vidioc_g_fmt_vid_cap    = heatmap_vidioc_fmt,
	.vidioc_try_fmt_vid_cap  = heatmap_vidioc_fmt,
	.vidioc_g_parm           = heatmap_vidioc_g_parm,

	.vidioc_enum_input       = heatmap_vidioc_enum_input,
	.vidioc_g_input          = heatmap_vidioc_g_input,
	.vidioc_s_input          = heatmap_vidioc_s_input,

	.vidioc_reqbufs          = vb2_ioctl_reqbufs,
	.vidioc_create_bufs      = vb2_ioctl_create_bufs,
	.vidioc_querybuf         = vb2_ioctl_querybuf,
	.vidioc_qbuf             = vb2_ioctl_qbuf,
	.vidioc_dqbuf            = vb2_ioctl_dqbuf,
	.vidioc_expbuf           = vb2_ioctl_expbuf,

	.vidioc_streamon         = vb2_ioctl_streamon,
	.vidioc_streamoff        = vb2_ioctl_streamoff,
};

static const struct video_device heatmap_video_device = {
	.name = "heatmap",
	.fops = &heatmap_video_fops,
	.ioctl_ops = &heatmap_video_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_TOUCH |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING,
};

int heatmap_probe(struct v4l2_heatmap *v4l2)
{
	int error;

	/* init channel to zero */
	heatmap_set_input(v4l2, 0);

	v4l2->frame = devm_kzalloc(v4l2->parent_dev,
		v4l2->format.sizeimage, GFP_KERNEL);
	if(!v4l2->frame) {
		error = -ENOMEM;
		goto err_probe;
	}

	/* register video device */
	strlcpy(v4l2->device.name, dev_name(v4l2->parent_dev),
		V4L2_DEVICE_NAME_SIZE);
	error = v4l2_device_register(v4l2->parent_dev, &v4l2->device);
	if (error)
		goto err_free_frame_storage;

	INIT_LIST_HEAD(&v4l2->heatmap_buffer_list);

	/* initialize the queue */
	spin_lock_init(&v4l2->heatmap_lock);
	mutex_init(&v4l2->lock);

	v4l2->queue = heatmap_queue;
	v4l2->queue.drv_priv = v4l2;
	v4l2->queue.lock = &v4l2->lock;

	error = vb2_queue_init(&v4l2->queue);
	if (error)
		goto err_unreg_v4l2;

	v4l2->vdev = heatmap_video_device;

	v4l2->vdev.v4l2_dev = &v4l2->device;
	v4l2->vdev.lock = &v4l2->lock;
	v4l2->vdev.vfl_dir = VFL_DIR_RX;
	v4l2->vdev.queue = &v4l2->queue;
	video_set_drvdata(&v4l2->vdev, v4l2);

	error = video_register_device(&v4l2->vdev, VFL_TYPE_TOUCH,
		FIRST_FREE_NODE);
	if (error)
		goto err_video_device_release;
	return 0;

err_video_device_release:
	video_device_release(&v4l2->vdev);

err_unreg_v4l2:
	v4l2_device_unregister(&v4l2->device);
err_free_frame_storage:
	kfree(v4l2->frame);
err_probe:
	return error;
}
EXPORT_SYMBOL(heatmap_probe);

void heatmap_remove(struct v4l2_heatmap *v4l2)
{
	video_unregister_device(&v4l2->vdev);
	v4l2_device_unregister(&v4l2->device);
	kfree(v4l2->frame);
}
EXPORT_SYMBOL(heatmap_remove);

MODULE_DESCRIPTION("Touchscreen heatmap video device");
MODULE_AUTHOR("Siarhei Vishniakou <svv@google.com>");
MODULE_LICENSE("GPL v2");

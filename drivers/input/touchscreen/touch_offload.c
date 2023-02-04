
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/input/touch_offload.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

static int touch_offload_open(struct inode *inode, struct file *file)
{
	struct touch_offload_context *context =
		container_of(inode->i_cdev, struct touch_offload_context, dev);

	file->private_data = context;
	pr_debug("%s\n", __func__);

	mutex_lock(&context->file_lock);
	if (context->file_in_use) {
		mutex_unlock(&context->file_lock);
		return -EBUSY;
	}
	context->file_in_use = true;
	mutex_unlock(&context->file_lock);

	/* Prepare context for consumer. E.g., event queue */
	return 0;
}

static int touch_offload_release(struct inode *inode, struct file *file)
{
	struct touch_offload_context *context = file->private_data;

	mutex_lock(&context->file_lock);
	context->file_in_use = false;
	mutex_unlock(&context->file_lock);

	pr_debug("%s\n", __func__);
	return 0;
}

static int pack_frame(struct touch_offload_context *context,
		      struct touch_offload_frame *frame, char **packed)
{
	/* TODO: compute precise size of a packed frame */
	int max_packed_frame_size =
		sizeof(struct touch_offload_frame) +
		TOUCH_OFFLOAD_DATA_SIZE_2D(context->caps.rx_size,
					   context->caps.tx_size) *
		MAX_CHANNELS;
	/* TODO: preallocate memory for a single packed frame */
	char *packed_mem = NULL;
	char *ptr = NULL;
	int channel_size;
	int i = 0;

	if (!frame || frame->num_channels > MAX_CHANNELS)
		return -EINVAL;

	packed_mem = kzalloc(max_packed_frame_size, GFP_KERNEL);
	if (packed_mem == NULL)
		return -ENOMEM;
	ptr = packed_mem;

	/* Copy the header */

	memcpy(ptr, &frame->header, sizeof(frame->header));
	ptr += sizeof(frame->header);

	/* Copy frame data */

	for (i = 0; i < frame->num_channels; i++) {
		if (frame->channel_type[i] == TOUCH_DATA_TYPE_COORD)
			channel_size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;
		else if ((frame->channel_type[i] & TOUCH_SCAN_TYPE_MUTUAL) != 0)
			channel_size =
			    TOUCH_OFFLOAD_FRAME_SIZE_2D(context->caps.rx_size,
							context->caps.tx_size);
		else if ((frame->channel_type[i] & TOUCH_SCAN_TYPE_SELF) != 0)
			channel_size =
			    TOUCH_OFFLOAD_FRAME_SIZE_1D(context->caps.rx_size,
							context->caps.tx_size);
		else {
			pr_err("%s: Invalid channel_type = 0x%08X", __func__,
			       frame->channel_type[i]);
			kfree(packed_mem);
			return -EINVAL;
		}
		memcpy(ptr, frame->channel_data[i], channel_size);
		ptr += channel_size;
	}

	*packed = packed_mem;
	return (ptr - packed_mem);
}

static ssize_t touch_offload_read(struct file *file, char __user *user_buffer,
				  size_t size, loff_t *offset)
{
	struct touch_offload_context *context = file->private_data;
	struct touch_offload_frame *frame;
	size_t copy_size;
	int result;
	unsigned long remaining;

	pr_debug("%s\n", __func__);

	if (context->num_buffers == 0)
		return -EINVAL;

	/* Block until touch events occur */
	/* Block (on completion?) until len >= 0. */
	/* Lock the event buffer */
	/* Copy contents of event buffer to service */
	/* Reset completion since len=0 */
	/* Unlock and return */

	/* If the end of the data is reached, free the packed_frame and
	 * return 0
	 */

	mutex_lock(&context->buffer_lock);

	if (context->packed_frame != NULL &&
	    *offset == context->packed_frame_size) {
		pr_err("%s: [Unexpected!] The buffer should have been recycled after the previous read.\n",
		       __func__);
		kfree(context->packed_frame);
		context->packed_frame = NULL;
		*offset = 0;
		mutex_unlock(&context->buffer_lock);
		return 0;
	} else if (context->packed_frame == NULL) {
		/* Process the next queued buffer */

		while (list_empty(&context->frame_queue)) {
			/* Presumably more buffers on the way, so block */
			mutex_unlock(&context->buffer_lock);

			/* Non-blocking read? */
			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;

			/* Block until data is available */
			if (wait_event_interruptible(context->read_queue,
					!list_empty(&context->frame_queue)))
				return -ERESTARTSYS;

			/* Check that the pipeline is still running */
			mutex_lock(&context->buffer_lock);
		}

		frame = list_entry(context->frame_queue.next,
				   struct touch_offload_frame, entry);
		list_del(&frame->entry);

		result = pack_frame(context, frame, &context->packed_frame);
		list_add_tail(&frame->entry, &context->free_pool);
		if (result <= 0) {
			pr_err("%s: Error packing frame! Result = %d.\n",
			       __func__, result);
			mutex_unlock(&context->buffer_lock);
			return -EINVAL;
		}
		if (result != context->packed_frame_size) {
			pr_err("%s: Packed frame size (%d) does not match size allocated per frame(%d)!\n",
			       __func__, result, context->packed_frame_size);
		}

	}

	/* Transfer the maximum amount of data */
	copy_size = min((long long)size, context->packed_frame_size - *offset);
	remaining = copy_to_user(user_buffer, context->packed_frame + *offset,
			      copy_size);
	if (remaining != 0)
		pr_err("%s: copy_to_user unexpectedly failed to copy %lu bytes.\n",
		       __func__, remaining);
	*offset += copy_size - remaining;

	/* Recycle the frame if transfer was complete */
	if (*offset == context->packed_frame_size) {
		kfree(context->packed_frame);
		context->packed_frame = NULL;
		*offset = 0;
	}

	mutex_unlock(&context->buffer_lock);

	return copy_size - remaining;
}

static unsigned int touch_offload_poll(struct file *file,
				       struct poll_table_struct *wait)
{
	struct touch_offload_context *context = file->private_data;
	unsigned int flags = 0;

	pr_debug("%s\n", __func__);

	poll_wait(file, &context->read_queue, wait);

	mutex_lock(&context->buffer_lock);

	if (context->packed_frame || !list_empty(&context->frame_queue))
		flags |= POLLIN | POLLRDNORM;

	mutex_unlock(&context->buffer_lock);
	return flags;
}

static int touch_offload_allocate_buffers(struct touch_offload_context *context,
					  int nb)
{
	struct touch_offload_frame *frame = NULL;
	int i;
	int num_channels;
	__u32 mask;
	__u32 size = 0;

	pr_debug("%s\n", __func__);

	num_channels = (context->config.read_coords ? 1 : 0) +
		       hweight_long(context->config.mutual_data_types) +
		       hweight_long(context->config.self_data_types);
	if (num_channels == 0 || num_channels > MAX_CHANNELS) {
		pr_err("%s: Configuration enables more (%d) than %d channels!\n",
		       __func__, num_channels, MAX_CHANNELS);
		return -EINVAL;
	}


	mutex_lock(&context->buffer_lock);

	/* Add new buffers to the free_pool */
	for (i = 0; i < nb; i++) {
		int chan = 0;
		struct touch_offload_frame *frame =
		    kzalloc(sizeof(struct touch_offload_frame), GFP_KERNEL);

		if (frame == NULL) {
			mutex_unlock(&context->buffer_lock);
			return -ENOMEM;
		}

		frame->header.frame_size = sizeof(frame->header);

		/* Allocate component buffers */

		if (context->config.read_coords) {
			struct TouchOffloadDataCoord *coord;

			frame->channel_type[chan] = TOUCH_DATA_TYPE_COORD;
			size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;
			frame->channel_data[chan] = kzalloc(size,
							       GFP_KERNEL);
			if (frame->channel_data[chan] == NULL)
				goto kzalloc_channel_fail;
			coord = (struct TouchOffloadDataCoord *)
					frame->channel_data[chan];
			coord->size_bytes = size;
			frame->channel_data_size[chan] = size;
			frame->header.frame_size += size;
			chan++;
		}
		for (mask = 0x01; mask < 0x100; mask <<= 1) {
			if ((context->config.mutual_data_types & mask) != 0) {
				struct TouchOffloadData2d *data;

				frame->channel_type[chan] =
				    TOUCH_SCAN_TYPE_MUTUAL | mask;
				size = TOUCH_OFFLOAD_FRAME_SIZE_2D(
						context->caps.rx_size,
						context->caps.tx_size);
				frame->channel_data[chan] = kzalloc(size,
								    GFP_KERNEL);
				if (frame->channel_data[chan] == NULL)
					goto kzalloc_channel_fail;
				data = (struct TouchOffloadData2d *)
						frame->channel_data[chan];
				data->size_bytes = size;
				pr_debug("%s: data->size_bytes = %u.\n",
					 __func__, data->size_bytes);
				frame->channel_data_size[chan] = size;
				frame->header.frame_size += size;
				chan++;
			}
		}
		for (mask = 0x01; mask < 0x100; mask <<= 1) {
			if ((context->config.self_data_types & mask) != 0) {
				struct TouchOffloadData1d *data;

				frame->channel_type[chan] =
					TOUCH_SCAN_TYPE_SELF | mask;
				size = TOUCH_OFFLOAD_FRAME_SIZE_1D(
						context->caps.rx_size,
						context->caps.tx_size);
				frame->channel_data[chan] = kzalloc(size,
								GFP_KERNEL);
				if (frame->channel_data[chan] == NULL)
					goto kzalloc_channel_fail;
				data = (struct TouchOffloadData1d *)
						frame->channel_data[chan];
				data->size_bytes = size;
				pr_debug("%s: data->size_bytes = %u.\n",
					 __func__, data->size_bytes);
				frame->channel_data_size[chan] = size;
				frame->header.frame_size += size;
				chan++;
			}
		}

		frame->num_channels = chan;

		if (context->packed_frame_size == 0)
			context->packed_frame_size = frame->header.frame_size;
		if (context->packed_frame_size != frame->header.frame_size)
			pr_err("%s: Frame size mismatch! %d != %d.\n", __func__,
			       context->packed_frame_size,
			       frame->header.frame_size);

		list_add_tail(&frame->entry, &context->free_pool);
		context->num_buffers++;
	}

	mutex_unlock(&context->buffer_lock);
	return 0;

kzalloc_channel_fail:
	/* Free all channels of "frame" before returning */
	if (frame)
		for (i = 0; i < MAX_CHANNELS; i++)
			kfree(frame->channel_data[i]);
	kfree(frame);

	mutex_unlock(&context->buffer_lock);
	return -ENOMEM;
}

static int touch_offload_free_buffers(struct touch_offload_context *context)
{
	int freed = 0;
	int chan = 0;

	pr_debug("%s\n", __func__);

	mutex_lock(&context->buffer_lock);

	/* Ensure there is no outstanding reserved_frame before continuing */
	while (context->reserved_frame != NULL) {
		mutex_unlock(&context->buffer_lock);
		wait_for_completion(&context->reserve_returned);
		mutex_lock(&context->buffer_lock);
	}

	if (context->num_buffers > 0) {
		while (!list_empty(&context->free_pool)) {
			struct list_head *next = context->free_pool.next;
			struct touch_offload_frame *frame =
			    list_entry(next, struct touch_offload_frame, entry);

			list_del(next);
			for (chan = 0; chan < frame->num_channels; chan++)
				kfree(frame->channel_data[chan]);
			kfree(frame);
			freed++;
		}

		while (!list_empty(&context->frame_queue)) {
			struct list_head *next = context->frame_queue.next;
			struct touch_offload_frame *frame =
			    list_entry(next, struct touch_offload_frame, entry);

			list_del(next);
			for (chan = 0; chan < frame->num_channels; chan++)
				kfree(frame->channel_data[chan]);
			kfree(frame);
			freed++;
		}
	}
	if (freed != context->num_buffers)
		pr_err("%s: mismatch between the number of buffers allocated(%d) and freed(%d)!",
		       __func__, context->num_buffers, freed);

	/* clean up */
	context->num_buffers = 0;
	context->reserved_frame = NULL;
	INIT_LIST_HEAD(&context->free_pool);
	INIT_LIST_HEAD(&context->frame_queue);
	context->packed_frame_size = 0;

	mutex_unlock(&context->buffer_lock);

	return 0;
}

static long touch_offload_ioctl(struct file *file, unsigned int ioctl_num,
				unsigned long ioctl_param)
{
	struct touch_offload_context *context = file->private_data;
	unsigned long err = 0;

	pr_debug("%s: ioctl_num=0x%08X, ioctl_param=0x%08lX\n", __func__,
		 ioctl_num, ioctl_param);

	switch (ioctl_num) {
	case TOUCH_OFFLOAD_IOC_RD_GETCAPS:
	{
		struct TouchOffloadIocGetCaps getCaps;

		/* Copy previously-populated caps */
		memcpy(&getCaps.caps, &context->caps,
		       sizeof(context->caps));

		err = copy_to_user((void *)ioctl_param, &getCaps,
				   sizeof(getCaps));
		if (err != 0) {
			pr_err("%s: copy_to_failed with err=0x%08lX",
			       __func__, err);
			return err;
		}
		break;
	}

	case TOUCH_OFFLOAD_IOC_WR_CONFIGURE:
	{
		struct TouchOffloadIocConfigure configure;
		int NUM_BUFFERS = 3;
		int num_channels;

		err = copy_from_user(&configure, (void *)ioctl_param,
				     sizeof(configure));
		if (err != 0) {
			pr_err("%s: copy_from_user failed with err=0x%08lX",
			       __func__, err);
			return err;
		}

		/* TODO: stop any active streaming */

		/* Purge any previously-allocated buffers */
		touch_offload_free_buffers(context);

		memset(&context->config, 0, sizeof(context->config));
		if ((configure.config.continuous_reporting &&
		     !context->caps.continuous_reporting) ||
		    (configure.config.noise_reporting &&
		     !context->caps.noise_reporting) ||
		    (configure.config.cancel_reporting &&
		     !context->caps.cancel_reporting) ||
		    (configure.config.filter_grip &&
		     !context->caps.filter_grip) ||
		    (configure.config.filter_palm &&
		     !context->caps.filter_palm)) {
			pr_err("%s: Invalid configuration enables unsupported features!\n",
			       __func__);
			err = -EINVAL;
			return err;
		} else if (configure.config.sensitivity_setting >=
			   context->caps.num_sensitivity_settings) {
			pr_err("%s: Invalid configuration enables unsupported sensitivity setting!\n",
			       __func__);
			err = -EINVAL;
			return err;
		} else if ((configure.config.mutual_data_types &
			    ~context->caps.touch_data_types) != 0 ||
			   (configure.config.self_data_types &
			    ~context->caps.touch_data_types) != 0) {
			pr_err("%s: Invalid configuration enables unsupported data types!\n",
			       __func__);
			err = -EINVAL;
			return err;
		}

		num_channels = (configure.config.read_coords ? 1 : 0) +
			       hweight_long(
				   configure.config.mutual_data_types) +
			       hweight_long(
				   configure.config.self_data_types);
		if (num_channels <= 0 || num_channels > MAX_CHANNELS) {
			pr_err("%s: Invalid configuration enables more (%d) than %d channels!\n",
			       __func__, num_channels, MAX_CHANNELS);
			err = -EINVAL;
			return err;
		}

		/* Copy sanitized config */
		memcpy(&context->config, &configure.config,
		       sizeof(context->config));

		/* Allocate frames */
		err = touch_offload_allocate_buffers(context, NUM_BUFFERS);
		if (err != 0) {
			pr_err("%s: failed to allocate buffers. err = 0x%08X.\n",
			       __func__, (unsigned int)err);
			return err;
		}

		break;
	}

	case TOUCH_OFFLOAD_IOC_WR_REPORT:
	{
		struct TouchOffloadIocReport report;

		err = copy_from_user(&report, (void *)ioctl_param,
				     sizeof(report));
		if (err != 0) {
			pr_err("%s: copy_from_user failed with err=0x%08lx.\n",
			       __func__, err);
			return err;
		}

		context->report_cb(context->hcallback, &report);
		break;
	}

	default:
		return -EINVAL;
	}

	return err;
}

const struct file_operations touch_offload_fops = {
	.owner	  = THIS_MODULE,
	.open	  = touch_offload_open,
	.release  = touch_offload_release,
	.read	  = touch_offload_read,
	.poll	  = touch_offload_poll,
	.unlocked_ioctl	  = touch_offload_ioctl
};

int touch_offload_reserve_frame(struct touch_offload_context *context,
				struct touch_offload_frame **frame)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	if (!context || !frame)
		return -EINVAL;
	*frame = NULL;

	mutex_lock(&context->buffer_lock);

	if (context->num_buffers == 0 || list_empty(&context->free_pool) ||
	    context->reserved_frame != NULL) {
		pr_debug("%s: buffer not available.\n", __func__);
		ret = -EINVAL;
	} else {
		reinit_completion(&context->reserve_returned);
		context->reserved_frame =
			list_entry(context->free_pool.next,
				   struct touch_offload_frame, entry);
		list_del(&context->reserved_frame->entry);
		*frame = context->reserved_frame;
	}

	mutex_unlock(&context->buffer_lock);

	return ret;
}
EXPORT_SYMBOL(touch_offload_reserve_frame);

int touch_offload_queue_frame(struct touch_offload_context *context,
			      struct touch_offload_frame *frame)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	if (!context || !frame)
		return -EINVAL;

	mutex_lock(&context->buffer_lock);

	if (context->num_buffers == 0 || frame != context->reserved_frame) {
		pr_err("%s: incorrect or NULL buffer submitted.\n", __func__);
		ret = -EINVAL;
	} else {
		/* TODO: Restore important constant fields */

		list_add_tail(&frame->entry, &context->frame_queue);
		context->reserved_frame = NULL;
		complete_all(&context->reserve_returned);

		wake_up(&context->read_queue);
	}

	mutex_unlock(&context->buffer_lock);

	return ret;
}
EXPORT_SYMBOL(touch_offload_queue_frame);

int touch_offload_init(struct touch_offload_context *context)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	/* Initialize ioctl interface */
	context->file_in_use = false;
	mutex_init(&context->file_lock);

	/* Initialize the buffer pool */
	context->num_buffers = 0;
	context->reserved_frame = NULL;
	INIT_LIST_HEAD(&context->free_pool);
	INIT_LIST_HEAD(&context->frame_queue);
	mutex_init(&context->buffer_lock);

	init_waitqueue_head(&context->read_queue);

	init_completion(&context->reserve_returned);
	complete_all(&context->reserve_returned);

	/* Initialize char device */
	context->major_num = register_chrdev(0, DEVICE_NAME,
					     &touch_offload_fops);
	if (context->major_num < 0) {
		pr_err("%s: register_chrdev failed with error = %u\n",
		       __func__, context->major_num);
		return context->major_num;
	}

	context->cls = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(context->cls)) {
		pr_err("%s: class_create failed with error = %ld.\n",
		       __func__, PTR_ERR(context->cls));
		unregister_chrdev(context->major_num, DEVICE_NAME);
		return PTR_ERR(context->cls);
	}

	context->device = device_create(context->cls, NULL,
					MKDEV(context->major_num, 0), NULL,
					DEVICE_NAME);
	if (IS_ERR(context->device)) {
		pr_err("%s: device_create failed with error = %ld.\n",
		       __func__, PTR_ERR(context->device));
		class_destroy(context->cls);
		unregister_chrdev(context->major_num, DEVICE_NAME);
		return PTR_ERR(context->device);
	}

	cdev_init(&context->dev, &touch_offload_fops);
	cdev_add(&context->dev, MKDEV(context->major_num, 0), 1);

	return ret;
}
EXPORT_SYMBOL(touch_offload_init);

int touch_offload_cleanup(struct touch_offload_context *context)
{
	pr_debug("%s\n", __func__);

	cdev_del(&context->dev);

	device_destroy(context->cls, MKDEV(context->major_num, 0));

	class_destroy(context->cls);

	unregister_chrdev(context->major_num, DEVICE_NAME);

	touch_offload_free_buffers(context);

	return 0;
}
EXPORT_SYMBOL(touch_offload_cleanup);

MODULE_DESCRIPTION("Touch Offload to AP");
MODULE_AUTHOR("Steve Pfetsch <spfetsch@google.com>");
MODULE_LICENSE("GPL v2");

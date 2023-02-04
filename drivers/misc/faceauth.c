/*
 * Google FaceAuth driver
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/faceauth.h>
#include <linux/faceauth_shared.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/module.h>
#include <misc/faceauth_hypx.h>

#include <linux/mfd/abc-pcie.h>
#include <linux/mfd/abc-pcie-notifier.h>

#include <linux/miscdevice.h>
#include <linux/rwsem.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <linux/workqueue.h>

#include <linux/mfd/abc-pcie-dma.h>
#include <uapi/linux/abc-pcie-dma.h>

#include <trace/events/systrace.h>

#define DEBUG_DATA_BIN_SIZE (2 * 1024 * 1024)
#define DEBUG_DATA_NUM_BINS 5

/* Timeout in ms */
#define FACEAUTH_TIMEOUT_MS 3000
#define M0_ENROLL_POLLING_PAUSE_US 50000
#define M0_AUTH_POLLING_PAUSE_US 15000
/* Polling interval in us */
#define M0_ENROLL_POLLING_INTERVAL_US 6000
#define M0_POLLING_INTERVAL_US 100000

struct faceauth_data {
	/*
	 * M0 Verbosity Level Encoding
	 *
	 * 64 bits wide allocated as follows:
	 * Bit  0   Errors
	 * Bits 1-3 Performance
	 * Bits 4-7 Scheduler
	 * Bits 8-11 IPU
	 * Bits 12-15 TPU
	 * Bits 16-19 Post Process
	 * Bits 20-63 Reserved
	 *
	 * In these slots, the debug levels are specified as follows:
	 * Level 0: 0b0000
	 * Level 1: 0b1000
	 * Level 2: 0b0100
	 * Level 3: 0b0010
	 * Level 4: 0b0001
	 *
	 * Level 0 means errors only. The other levels yield increasingly more
	 * information.
	 *
	 * To set all these levels, you must write the number in either
	 * unsigned hexadecimal format or unisigned decimal format
	 * to a certain file: /d/faceauth/m0_verbosity_level
	 * If using hexadecimal, you need to put "0x" in front.
	 * For example:
	 * either
	 * adb shell "echo 0x108248 > /d/faceauth/m0_verbosity_level"
	 * or
	 * adb shell "echo 1081928 > /d/faceauth/m0_verbosity_level"
	 * will result in the following settings:
	 * general errors level 0 (meaning ON)
	 * performance level 2
	 * scheduler level 3
	 * IPU level 1
	 * TPU level 0
	 * post process level 4
	 */
	uint64_t m0_verbosity_level;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct dentry *debugfs_root;
#endif
	bool can_transfer; /* Guarded by rwsem */
	uint64_t retry_count;
	atomic_t in_use;
	struct rw_semaphore rwsem;
	struct miscdevice misc_dev;
	struct device *device;
	struct delayed_work listener_init;
	struct notifier_block pcie_link_blocking_nb;
	bool is_secure_camera;
	bool debug_enabled;
};

static int process_cache_flush_idxs(int16_t *flush_idxs, uint32_t flush_size);
static void clear_debug_data(void);
static void move_debug_data_to_tail(void);
static void enqueue_debug_data(struct faceauth_data *data, uint32_t ab_result);
static int dequeue_debug_data(struct faceauth_debug_data *debug_step_data);

struct {
	int head_idx;
	int tail_idx;
	int count;
	char *data_buffer;
} debug_data_queue;

static long faceauth_dev_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	int err = 0;
	int polling_interval = M0_POLLING_INTERVAL_US;
	struct faceauth_start_data start_step_data = { 0 };
	struct faceauth_init_data init_step_data = { 0 };
	unsigned long stop;
	bool send_images_data;
	struct faceauth_data *data = file->private_data;
	bool need_trace_end = false;
	struct faceauth_debug_data debug_step_data;
	unsigned int polling_pause = M0_AUTH_POLLING_PAUSE_US;
	down_read(&data->rwsem);
	if (!data->can_transfer && cmd != FACEAUTH_DEV_IOC_DEBUG_DATA) {
		err = -EIO;
		pr_info("Cannot do transfer due to link down\n");
		goto exit;
	}

	switch (cmd) {
	case FACEAUTH_DEV_IOC_INIT:
		pr_info("el2: faceauth init IOCTL\n");

		if (copy_from_user(&init_step_data, (const void __user *)arg,
				   sizeof(init_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		err = el2_faceauth_wait_pil_dma_over();
		if (err < 0)
			goto exit;

		data->is_secure_camera =
			init_step_data.features & SECURE_CAMERA_DATA;

		err = el2_faceauth_init(data->device, &init_step_data,
					data->m0_verbosity_level);
		if (err < 0)
			goto exit;
		break;
	case FACEAUTH_DEV_IOC_START:
		pr_info("el2: faceauth start IOCTL\n");

		if (copy_from_user(&start_step_data, (const void __user *)arg,
				   sizeof(start_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		send_images_data =
			start_step_data.operation == COMMAND_ENROLL ||
			start_step_data.operation == COMMAND_VALIDATE;
		if (send_images_data) {
			if (!start_step_data.image_dot_left_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_dot_right_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_flood_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_flood_right_size) {
				err = -EINVAL;
				goto exit;
			}
		}

		err = process_cache_flush_idxs(
			start_step_data.cache_flush_indexes,
			start_step_data.cache_flush_size);
		if (err)
			goto exit;

		ATRACE_BEGIN("el2_faceauth_process");
		err = el2_faceauth_process(
			data->device, &start_step_data,
			data->is_secure_camera);
		ATRACE_END();
		if (err)
			goto exit;

		/* Check completion flag */
		pr_info("Waiting for completion.\n");
		if (start_step_data.operation == COMMAND_ENROLL) {
			polling_pause = M0_ENROLL_POLLING_PAUSE_US;
			polling_interval = M0_ENROLL_POLLING_INTERVAL_US;
		}
		ATRACE_BLOCK("M0_POLLING_PAUSE_US",
			usleep_range(polling_pause, polling_pause + 1)
		);
		stop = jiffies + msecs_to_jiffies(FACEAUTH_TIMEOUT_MS);
		need_trace_end = true;
		ATRACE_BEGIN("get_faceauth_process_result");
		for (;;) {
			err = el2_faceauth_get_process_result(data->device,
							      &start_step_data);

			if (err) {
				pr_err("Failed to get results from EL2 %d\n",
				       err);
				goto exit;
			}

			if (start_step_data.result !=
			    WORKLOAD_STATUS_NO_STATUS) {
				/* We've got a non-zero status from AB executor
				 * Faceauth processing is completed
				 */
				break;
			}
			if (time_before(stop, jiffies)) {
				if (start_step_data.ab_exception_number)
					err = -EREMOTEIO;
				else
					err = -ETIME;

				if (data->debug_enabled)
					enqueue_debug_data(
						data,
						WORKLOAD_STATUS_NO_STATUS);
				goto exit;
			}

			usleep_range(polling_interval, polling_interval + 1);
			polling_interval = polling_interval > 1000 ?
						   polling_interval >> 2 :
						   1000;
		}
		ATRACE_END();
		ATRACE_BEGIN("copy_faceauth_result_to_user");
		if (data->debug_enabled)
			enqueue_debug_data(data, start_step_data.result);
		if (copy_to_user((void __user *)arg, &start_step_data,
				 sizeof(start_step_data))) {
			err = -EFAULT;
			goto exit;
		}
		ATRACE_END();
		need_trace_end = false;
		break;
	case FACEAUTH_DEV_IOC_CLEANUP:
		/* In case of EL2 cleanup happens in PIL callback */
		/* TODO cleanup Airbrush DRAM */
		pr_info("el2: faceauth cleanup IOCTL\n");
		el2_faceauth_cleanup(data->device);
		data->is_secure_camera = false;
		break;
	case FACEAUTH_DEV_IOC_DEBUG:
		if (!data->debug_enabled) {
			err = -EOPNOTSUPP;
			break;
		}
		pr_info("el2: faceauth debug log IOCTL\n");
		if (copy_from_user(&debug_step_data, (const void __user *)arg,
				   sizeof(debug_step_data))) {
			err = -EFAULT;
			goto exit;
		}
		ATRACE_BEGIN("el2_faceauth_gather_debug_log");
		err = el2_faceauth_gather_debug_log(
			data->device, &debug_step_data);
		ATRACE_END();
		break;
	case FACEAUTH_DEV_IOC_DEBUG_DATA:
		if (!data->debug_enabled) {
			err = -EOPNOTSUPP;
			break;
		}
		pr_info("el2: faceauth debug data IOCTL\n");
		if (copy_from_user(&debug_step_data, (const void __user *)arg,
				   sizeof(debug_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		if (debug_step_data.debug_buffer_size < DEBUG_DATA_BIN_SIZE) {
			err = -EINVAL;
			goto exit;
		}

		switch (debug_step_data.flags) {
		case FACEAUTH_GET_DEBUG_DATA_FROM_FIFO:
			err = dequeue_debug_data(&debug_step_data);
			break;
		case FACEAUTH_GET_DEBUG_DATA_MOST_RECENT:
			move_debug_data_to_tail();
			err = dequeue_debug_data(&debug_step_data);
			break;
		case FACEAUTH_GET_DEBUG_DATA_FROM_AB_DRAM:
			if (!data->can_transfer) {
				pr_info("Cannot do transfer due to link down\n");
				err = -EIO;
				goto exit;
			}
			clear_debug_data();
			enqueue_debug_data(data, WORKLOAD_STATUS_NO_STATUS);
			err = dequeue_debug_data(&debug_step_data);
			break;
		default:
			err = -EINVAL;
			break;
		}
		break;
	default:
		err = -EOPNOTSUPP;
		goto exit;
	}

exit:
	if (need_trace_end)
		ATRACE_END();
	up_read(&data->rwsem);
	return err;
}

static int faceauth_open(struct inode *inode, struct file *file)
{
	struct miscdevice *m = file->private_data;
	struct faceauth_data *data =
		container_of(m, struct faceauth_data, misc_dev);

	file->private_data = data;
	if (atomic_cmpxchg(&data->in_use, 0, 1))
		return -EBUSY;
	return 0;
}

static int faceauth_release(struct inode *inode, struct file *file)
{
	struct faceauth_data *data = file->private_data;
	atomic_set(&data->in_use, 0);
	return 0;
}

static const struct file_operations faceauth_dev_operations = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = faceauth_dev_ioctl,
	.compat_ioctl = faceauth_dev_ioctl,
	.open = faceauth_open,
	.release = faceauth_release,
};

static int process_cache_flush_idxs(int16_t *flush_idxs, uint32_t flush_size)
{
	int i;

	if (flush_size > FACEAUTH_MAX_CACHE_FLUSH_SIZE) {
		pr_err("Wrong cache flush size\n");
		return -EINVAL;
	}

	if (flush_size < FACEAUTH_MAX_CACHE_FLUSH_SIZE) {
		flush_idxs[flush_size] = -1;
	}

	for (i = 0; i < flush_size; ++i) {
		if (flush_idxs[i] < 0
		    || flush_idxs[i] >= MAX_CACHE_ENROLLMENT) {
			pr_err("Wrong cache flush index\n");
			return -EINVAL;
		}
	}

	return 0;
}

static void enqueue_debug_data(struct faceauth_data *data, uint32_t ab_result)
{
	void *bin_addr;
	int err;
	struct faceauth_debug_entry *debug_entry;

	bin_addr = debug_data_queue.data_buffer +
		   (DEBUG_DATA_BIN_SIZE * debug_data_queue.head_idx);
	err = el2_gather_debug_data(data->device, bin_addr,
				    DEBUG_DATA_BIN_SIZE);

	if (err) {
		pr_err("Debug data gathering failed: %d\n", err);
		return;
	}

	debug_data_queue.head_idx =
		(debug_data_queue.head_idx + 1) % DEBUG_DATA_NUM_BINS;

	if (debug_data_queue.count == DEBUG_DATA_NUM_BINS) {
		debug_data_queue.tail_idx =
			(debug_data_queue.tail_idx + 1) % DEBUG_DATA_NUM_BINS;
	} else {
		debug_data_queue.count++;
	}

	debug_entry = bin_addr;
	debug_entry->status = ab_result;
	do_gettimeofday(&(debug_entry->timestamp));
}

static void move_debug_data_to_tail(void)
{
	int delta;

	if (debug_data_queue.count <= 1)
		return;

	delta = debug_data_queue.count - 1;
	debug_data_queue.count -= delta;
	debug_data_queue.tail_idx =
		(debug_data_queue.tail_idx + delta) % DEBUG_DATA_NUM_BINS;
}

static void clear_debug_data(void)
{
	debug_data_queue.count = 0;
	debug_data_queue.head_idx = 0;
	debug_data_queue.tail_idx = 0;
}

static int dequeue_debug_data(struct faceauth_debug_data *debug_step_data)
{
	void *bin_addr;

	if (debug_data_queue.count == 0) {
		return -ENODATA;
	}

	if (debug_step_data->debug_buffer_size < DEBUG_DATA_BIN_SIZE) {
		return -ENOBUFS;
	}

	bin_addr = debug_data_queue.data_buffer +
		   (DEBUG_DATA_BIN_SIZE * debug_data_queue.tail_idx);
	debug_data_queue.tail_idx =
		(debug_data_queue.tail_idx + 1) % DEBUG_DATA_NUM_BINS;
	debug_data_queue.count--;

	if (copy_to_user((void __user *)(debug_step_data->debug_buffer),
			 bin_addr, DEBUG_DATA_BIN_SIZE)) {
		return -EFAULT;
	}

	return 0;
}

static void faceauth_link_listener_init(struct work_struct *work)
{
	struct faceauth_data *data =
		container_of(work, struct faceauth_data, listener_init.work);
	int err;

	err = abc_register_pcie_link_blocking_event(
		&data->pcie_link_blocking_nb);

	/* TODO: Use retry count to dynamiclly adjust retry timeout */
	if (err == -EAGAIN) {
		if (data->retry_count % 50 == 0)
			pr_info("Retry faceauth link init later.\n");
		data->retry_count++;
		schedule_delayed_work(&data->listener_init,
				      msecs_to_jiffies(1000));
	} else if (err)
		pr_err("CANNOT init link listened event in faceauth driver, err code %d\n",
		       err);
	else
		pr_info("Successfully register link listener for faceauth driver");
	return;
}

static int faceauth_pcie_blocking_listener(struct notifier_block *nb,
					   unsigned long action, void *data)
{
	struct faceauth_data *dev_data =
		container_of(nb, struct faceauth_data, pcie_link_blocking_nb);

	if (action & ABC_PCIE_LINK_ENTER_EL2) {
		down_read(&dev_data->rwsem);
		if (!dev_data->can_transfer)
			pr_err("ERROR: Wrong state, receive ENTER_EL2 while link down");
		up_read(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_EXIT_EL2) {
		down_read(&dev_data->rwsem);
		if (!dev_data->can_transfer)
			pr_err("ERROR: Wrong state, receive EXIT_EL2 while link down");
		up_read(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_ERROR) {
		/*
		 * Take a reader lock and update the flag as soon as possible.
		 */
		down_read(&dev_data->rwsem);
		dev_data->can_transfer = false;
		up_read(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_PRE_DISABLE) {
		/* Use the writer lock to prevent any incoming reader */
		down_write(&dev_data->rwsem);
		dev_data->can_transfer = false;
		pr_info("All ongoing ioctls are finished, confirm disable");
		up_write(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_POST_ENABLE) {
		/*
		 * Under this scenerio, this is actually a reader
		 * There's no need to block any other reader since
		 * they'll bail out when they got the value.
		 */
		down_read(&dev_data->rwsem);
		dev_data->can_transfer = true;
		up_read(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

#if IS_ENABLED(CONFIG_DEBUG_FS)

static int faceauth_m0_verbosity_set(void *ptr, u64 val)
{
	struct faceauth_data *data = ptr;

	data->m0_verbosity_level = val;
	return 0;
}

static int faceauth_m0_verbosity_get(void *ptr, u64 *val)
{
	struct faceauth_data *data = ptr;

	*val = data->m0_verbosity_level;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_m0_verbosity, faceauth_m0_verbosity_get,
			 faceauth_m0_verbosity_set, "0x%016llx\n");

static int faceauth_debug_enabled_set(void *ptr, u64 val)
{
	struct faceauth_data *data = ptr;

	data->debug_enabled = !!val;
	return 0;
}

static int faceauth_debug_enabled_get(void *ptr, u64 *val)
{
	struct faceauth_data *data = ptr;

	*val = data->debug_enabled;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_debug_enabled, faceauth_debug_enabled_get,
			 faceauth_debug_enabled_set, "%llu\n");

static void faceauth_debugfs_init(struct faceauth_data *data)
{
	struct dentry *debugfs_root;
	struct dentry *m0_verbosity_level;
	struct dentry *debug_enabled;
	int err = 0;

	debugfs_root = debugfs_create_dir("faceauth", NULL);
	if (IS_ERR_OR_NULL(debugfs_root)) {
		pr_err("Failed to create faceauth debugfs");
		err = -EIO;
		goto exit;
	}
	data->debugfs_root = debugfs_root;

	m0_verbosity_level =
		debugfs_create_file("m0_verbosity_level", 0660, debugfs_root,
				    data, &fops_m0_verbosity);
	if (!m0_verbosity_level) {
		err = -EIO;
		goto exit;
	}

	debug_enabled =
		debugfs_create_file("debug_enabled", 0660, debugfs_root,
				    data, &fops_debug_enabled);
	if (!debug_enabled) {
		err = -EIO;
		goto exit;
	}

	return;

exit:
	debugfs_remove_recursive(debugfs_root);
	data->debugfs_root = NULL;

	pr_err("faceauth debugfs initialization failed: %d\n", err);
}

static void faceauth_debugfs_remove(struct faceauth_data *data)
{
	if (!data->debugfs_root)
		return;

	debugfs_remove_recursive(data->debugfs_root);
}

#else /* CONFIG_DEBUG_FS */

static void faceauth_debugfs_init(struct faceauth_data *data)
{
}

static void faceauth_debugfs_remove(struct faceauth_data *data)
{
}

#endif /* CONFIG_DEBUG_FS */

static int faceauth_probe(struct platform_device *pdev)
{
	int err;
	struct faceauth_data *data;
	int i;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	platform_set_drvdata(pdev, data);

	init_rwsem(&data->rwsem);
	data->device = &pdev->dev;
	data->can_transfer = true;
	data->retry_count = 0;
	data->misc_dev.minor = MISC_DYNAMIC_MINOR,
	data->misc_dev.name = "faceauth",
	data->misc_dev.fops = &faceauth_dev_operations;
	data->pcie_link_blocking_nb.notifier_call =
		faceauth_pcie_blocking_listener;

	INIT_DELAYED_WORK(&data->listener_init, faceauth_link_listener_init);

	schedule_delayed_work(&data->listener_init, msecs_to_jiffies(1000));

	err = misc_register(&data->misc_dev);
	if (err)
		goto exit1;

	faceauth_debugfs_init(data);

	el2_faceauth_probe(data->device);

	clear_debug_data();
	debug_data_queue.data_buffer =
		vmalloc(DEBUG_DATA_BIN_SIZE * DEBUG_DATA_NUM_BINS);

	if (debug_data_queue.data_buffer == NULL) {
		err = -ENOMEM;
		goto exit3;
	}

	for (i = 0; i < DEBUG_DATA_NUM_BINS; i++) {
		memset(debug_data_queue.data_buffer + (DEBUG_DATA_BIN_SIZE * i),
		       0, sizeof(struct faceauth_debug_entry));
	}

	data->debug_enabled = true;
	return 0;

exit3:
	faceauth_debugfs_remove(data);
	misc_deregister(&data->misc_dev);

exit1:
	abc_unregister_pcie_link_blocking_event(&data->pcie_link_blocking_nb);
	return err;
}

static int faceauth_remove(struct platform_device *pdev)
{
	struct faceauth_data *data = platform_get_drvdata(pdev);

	el2_faceauth_remove(data->device);
	abc_unregister_pcie_link_blocking_event(&data->pcie_link_blocking_nb);
	misc_deregister(&data->misc_dev);
	faceauth_debugfs_remove(data);
	clear_debug_data();
	vfree(debug_data_queue.data_buffer);
	return 0;
}

static struct platform_driver faceauth_driver = {
	.probe = faceauth_probe,
	.remove = faceauth_remove,
	.driver = {
		.name = "faceauth",
		.owner = THIS_MODULE,
	},
};
struct platform_device *faceauth_pdev;

static int __init faceauth_init(void)
{
	int ret;

	faceauth_pdev =
		platform_device_register_simple("faceauth", -1, NULL, 0);
	if (IS_ERR(faceauth_pdev))
		return PTR_ERR(faceauth_pdev);
	arch_setup_dma_ops(&faceauth_pdev->dev, 0, U64_MAX, NULL, false);
	ret = dma_coerce_mask_and_coherent(&faceauth_pdev->dev,
					   DMA_BIT_MASK(47));
	if (ret) {
		pr_err("Can't set DMA mask for faceauth device: %d\n", ret);
		goto error;
	}

	ret = platform_driver_register(&faceauth_driver);
	if (ret) {
		pr_err("Can't register Faceauth driver: %d\n", ret);
		goto error;
	}

	return 0;
error:
	platform_device_unregister(faceauth_pdev);
	return ret;
}
module_init(faceauth_init);

static void __exit faceauth_exit(void)
{
	platform_driver_unregister(&faceauth_driver);
	platform_device_unregister(faceauth_pdev);
}
module_exit(faceauth_exit);

MODULE_AUTHOR("Anatol Pomazau <anatol@google.com>, Lei Liu <leliu@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Google FaceAuth driver");

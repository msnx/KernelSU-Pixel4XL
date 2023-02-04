
#ifndef TOUCH_OFFLOAD_H
#define TOUCH_OFFLOAD_H

#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/completion.h>

#include <uapi/input/touch_offload.h>

/* Maximum number of channels of touch data */
#define MAX_CHANNELS 5

#define DEVICE_NAME "touch_offload"
#define CLASS_NAME "touch_offload"

/* Frame of touch data
 *
 * entry - list entry in either the free pool or the event queue
 * header - frame header, including timestamp and index
 * num_channels - number of channels in this frame
 * channel_type - data type of each channel
 * channel_data_size - size of the data in each channel
 * channel_data - raw channel data
 */
struct touch_offload_frame {
	struct list_head entry;
	struct TouchOffloadFrameHeader header;
	int num_channels;
	int channel_type[MAX_CHANNELS];
	__u32 channel_data_size[MAX_CHANNELS];
	__u8 *channel_data[MAX_CHANNELS];
};

/* Touch Offload Context
 *
 * dev - char device
 * major_num - device major number
 * cls - pointer to class associated class
 * device - pointer to associated device
 * file - char device file for ioctl interface
 * file_lock - mutex for the ioctl interface
 * file_in_use - flag indicating the ioctl interface in use by one client
 * caps - capabilities supported by the touch driver
 * config - configuration in use by the touch driver
 * coords - snapshot of the current coordinate state
 * num_buffers - size of the pool of frame buffers
 * free_pool - list of buffers available for use
 * frame_queue - list of captured frames queued for the service
 * reserved_frame - buffer ready to be filled with the next touch frame
 * read_queue - waitqueue for blocked readers
 * packed_frame - serialized frame being read by the char device client
 * packed_frame_size - size of the array pointed to by packed_frame
 * buffer_lock - mutex protecting buffer management
 * reserve_returned - indicates that the reserved buffer was released
 * hcallback - handle/pointer to driver's private callback context
 * report_cb - driver callback used to report touch events
 * offload_running - indicates whether the offload path is in use
 */
struct touch_offload_context {
	/* ioctl interface */
	struct cdev dev;
	int major_num;
	struct class *cls;
	struct device *device;
	struct file file;
	struct mutex file_lock;
	bool file_in_use;

	/* touch capabilities */
	struct TouchOffloadCaps caps;

	/* touch capture configuration */
	struct TouchOffloadConfig config;

	/* coordinate state */
	struct TouchOffloadCoord coords[MAX_COORDS];

	/* buffer management */
	int num_buffers;
	struct list_head free_pool;
	struct list_head frame_queue;
	struct touch_offload_frame *reserved_frame;
	wait_queue_head_t read_queue;
	char *packed_frame;
	__u32 packed_frame_size;
	struct mutex buffer_lock;
	struct completion reserve_returned;

	/* callbacks */
	void *hcallback;
	void (*report_cb)(void *hcallback,
			  struct TouchOffloadIocReport *report);

	int offload_running;
};

/* Initialize the touch_offload driver */
int touch_offload_init(struct touch_offload_context *context);

/* Clean-up the touch_offload driver */
int touch_offload_cleanup(struct touch_offload_context *context);

/* Allocate a new empty frame from the free pool */
int touch_offload_reserve_frame(struct touch_offload_context *context,
				struct touch_offload_frame **frame);

/* Insert a populated frame into the queue */
int touch_offload_queue_frame(struct touch_offload_context *context,
			      struct touch_offload_frame *frame);

#endif // TOUCH_OFFLOAD_H

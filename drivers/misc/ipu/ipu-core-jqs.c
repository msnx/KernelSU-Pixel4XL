/*
 * JQS management support for the Paintbox programmable IPU
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

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "ipu-adapter.h"
#include "ipu-core-jqs.h"
#include "ipu-core-jqs-msg-transport.h"
#include "ipu-core-jqs-preamble.h"
#include "ipu-regs.h"

#define JQS_FIRMWARE_NAME "paintbox-jqs.fw"

#define JQS_LOG_LEVEL_DEF         JQS_LOG_LEVEL_ERROR
#define JQS_LOG_TRIGGER_LEVEL_DEF JQS_LOG_LEVEL_ERROR
#define JQS_LOG_SINK_DEF          JQS_LOG_SINK_MESSAGE
#define JQS_LOG_UART_BAUD_DEF     115200

#define JQS_SHUTDOWN_TIMEOUT_INTERVAL_US	100
#define JQS_SHUTDOWN_TIMEOUT_INTERVAL_US_LOWER	90
#define JQS_SHUTDOWN_TIMEOUT_INTERVAL_US_UPPER	110
#define JQS_SHUTDOWN_TIMEOUT_LIMIT_US		1000

#define A0_IPU_DEFAULT_CLOCK_RATE 549000000 /* hz */

/* Delay for I/O block to wake up */
#define IO_POWER_RAMP_TIME 10 /* us */

/* Delay to prevent in-rush current */
#define CORE_POWER_RAMP_TIME 10 /* us */

/* Delay for rams to wake up */
#define RAM_POWER_RAIL_RAMP_TIME 1 /* us */

/* Delay for system to stabilize before sending real traffic */
#define CORE_SYSTEM_STABLIZE_TIME 100 /* us */


static ssize_t jqs_build_number_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct paintbox_bus *bus = ipu_bus_from_device(dev);

	if (bus->jqs.valid_versions)
		return scnprintf(buf, PAGE_SIZE, "%u\n", bus->jqs.build_number);
	else
		return scnprintf(buf, PAGE_SIZE, "NA\n");
}

static ssize_t jqs_message_version_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct paintbox_bus *bus = ipu_bus_from_device(dev);

	if (bus->jqs.valid_versions)
		return scnprintf(buf, PAGE_SIZE, "%u\n",
				bus->jqs.message_version);
	else
		return scnprintf(buf, PAGE_SIZE, "NA\n");
}
static ssize_t jqs_command_version_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct paintbox_bus *bus = ipu_bus_from_device(dev);

	if (bus->jqs.valid_versions)
		return scnprintf(buf, PAGE_SIZE, "%u\n",
				bus->jqs.command_version);
	else
		return scnprintf(buf, PAGE_SIZE, "NA\n");
}

static struct device_attribute jqs_attrs[] = {
	__ATTR_RO(jqs_build_number),
	__ATTR_RO(jqs_message_version),
	__ATTR_RO(jqs_command_version),
};

static int ipu_core_jqs_create_sysfs(struct device *dev)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(jqs_attrs); i++) {
		ret = device_create_file(dev, &jqs_attrs[i]);

		if (WARN_ON(ret))
			return ret;
	}

	return 0;
}

static void ipu_core_jqs_remove_sysfs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(jqs_attrs); i++)
		device_remove_file(dev, &jqs_attrs[i]);
}

static inline bool ipu_core_jqs_is_clock_ready(struct paintbox_bus *bus)
{
	return bus->jqs.clock_rate_hz > 0;
}

int ipu_core_jqs_send_clock_rate(struct paintbox_bus *bus,
		uint32_t clock_rate_hz)
{
	struct jqs_message_clock_rate req;

	dev_dbg(bus->parent_dev, "%s: clock rate %u\n", __func__,
			clock_rate_hz);

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_CLOCK_RATE);

	req.clock_rate = clock_rate_hz;

	return ipu_core_jqs_msg_transport_kernel_write(bus,
			(const struct jqs_message *)&req);
}

/* The caller to this function must hold bus->jqs.lock */
int ipu_core_jqs_send_set_log_info(struct paintbox_bus *bus)
{
	struct jqs_message_set_log_info req;

	dev_dbg(bus->parent_dev,
			"%s: log sinks 0x%08x log level %u log int level %u uart baud_rate %u\n",
			__func__, bus->jqs.log_sink_mask, bus->jqs.log_level,
			bus->jqs.log_trigger_level, bus->jqs.uart_baud);

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_SET_LOG_INFO);

	req.log_level = bus->jqs.log_level;
	req.interrupt_level = bus->jqs.log_trigger_level;
	req.log_sinks = bus->jqs.log_sink_mask;
	req.uart_baud_rate = bus->jqs.uart_baud;

	return ipu_core_jqs_msg_transport_kernel_write(bus,
			(const struct jqs_message *)&req);
}

int ipu_core_jqs_send_shutdown_mode(struct paintbox_bus *bus,
		enum jqs_shutdown_mode shutdown_mode)
{
	struct jqs_message_shutdown_mode req;
	struct jqs_message_ack rsp;

	req.shutdown_mode = shutdown_mode;

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_SHUTDOWN_MODE);

	return ipu_core_jqs_msg_transport_kernel_write_sync(bus,
			(const struct jqs_message *)&req,
			(struct jqs_message *)&rsp, sizeof(rsp));
}

/* The caller to this function must hold bus->jqs.lock */
int ipu_core_jqs_load_firmware(struct paintbox_bus *bus)
{
	int ret;

	if (bus->jqs.status > JQS_FW_STATUS_INIT)
		return 0;

	dev_dbg(bus->parent_dev, "requesting firmware %s\n", JQS_FIRMWARE_NAME);

	ret = request_firmware(&bus->jqs.fw, JQS_FIRMWARE_NAME,
			bus->parent_dev);
	if (ret < 0) {
		dev_err(bus->parent_dev, "%s: unable to load %s, %d\n",
				__func__, JQS_FIRMWARE_NAME, ret);
		return ret;
	}

	bus->jqs.status = JQS_FW_STATUS_REQUESTED;

	return 0;
}

/* The caller to this function must hold bus->jqs.lock */
void ipu_core_jqs_unload_firmware(struct paintbox_bus *bus)
{
	if (bus->jqs.status != JQS_FW_STATUS_REQUESTED)
		return;

#if IS_ENABLED(CONFIG_IPU_DEBUG)
	if (bus->jqs.status_min >= JQS_FW_STATUS_REQUESTED)
		return;
#endif

	dev_dbg(bus->parent_dev, "%s: unloading firmware\n", __func__);

	if (bus->jqs.fw) {
		release_firmware(bus->jqs.fw);
		bus->jqs.fw = NULL;
	}

	bus->jqs.valid_versions = false;
	bus->jqs.status = JQS_FW_STATUS_INIT;
}

/* The caller to this function must hold bus->jqs.lock */
int ipu_core_jqs_stage_firmware(struct paintbox_bus *bus)
{
	struct jqs_firmware_preamble preamble;
	size_t fw_binary_len_bytes;

	if (bus->jqs.status > JQS_FW_STATUS_STAGED)
		return 0;

	if (WARN_ON(!bus->jqs.fw))
		return -EINVAL;

	if (!ipu_core_is_ready(bus)) {
		dev_err(bus->parent_dev,
				"%s: unable to stage JQS firmware, hw not ready\n",
				__func__);
		return -ENETDOWN;
	}

	memcpy(&preamble, bus->jqs.fw->data, min(sizeof(preamble),
			bus->jqs.fw->size));

	if (preamble.magic != JQS_PREAMBLE_MAGIC_WORD) {
		dev_err(bus->parent_dev,
			"%s: invalid magic in JQS firmware preamble\n",
			__func__);
		return -EINVAL;
	}

	if (sizeof(preamble) == preamble.size) {
		if (preamble.message_version != JQS_MESSAGE_VERSION) {
			dev_err(bus->parent_dev,
				"%s: invalid message version in JQS firmware preamble\n",
				__func__);
			return -EINVAL;
		}

		bus->jqs.valid_versions = true;
		bus->jqs.build_number = preamble.build_number;
		bus->jqs.message_version = preamble.message_version;
		bus->jqs.command_version = preamble.command_version;

		dev_dbg(bus->parent_dev,
				"%s: size %u fw_base_address 0x%08x FW and working set size %u prefill transport offset bytes %u message version %u command version %u\n",
				__func__, preamble.size,
				preamble.fw_base_address,
				preamble.fw_and_working_set_bytes,
				preamble.prefill_transport_offset_bytes,
				preamble.message_version,
				preamble.command_version);
	} else {
		dev_dbg(bus->parent_dev,
				"%s: size %u fw_base_address 0x%08x FW and working set size %u prefill transport offset bytes %u\n",
				__func__, preamble.size,
				preamble.fw_base_address,
				preamble.fw_and_working_set_bytes,
				preamble.prefill_transport_offset_bytes);
	}

	/* TODO(b/115524239):  It would be good to have some sort of bounds
	 * checking to make sure that the firmware could not allocate an
	 * unreasonable amount of memory for its working set.
	 *
	 * TODO(b/115522126):  The firmware is compiled for a specific address
	 * in AB DRAM.  This will necessitate having a carveout region in AB
	 * DRAM so we can guarantee the address.
	 */
	bus->jqs.fw_shared_buffer = bus->ops->alloc_shared_memory(
			bus->parent_dev,
			preamble.fw_and_working_set_bytes);
	if (IS_ERR_OR_NULL(bus->jqs.fw_shared_buffer))
		return PTR_ERR(bus->jqs.fw_shared_buffer);

	fw_binary_len_bytes = bus->jqs.fw->size - preamble.size;

	memcpy(bus->jqs.fw_shared_buffer->host_vaddr, bus->jqs.fw->data +
			preamble.size,
			fw_binary_len_bytes);

	bus->ops->sync_shared_memory(bus->parent_dev,
			bus->jqs.fw_shared_buffer, 0,
			fw_binary_len_bytes, DMA_TO_DEVICE);

	bus->jqs.status = JQS_FW_STATUS_STAGED;

	return 0;
}

/* The caller to this function must hold bus->jqs.lock */
void ipu_core_jqs_unstage_firmware(struct paintbox_bus *bus)
{
	if (bus->jqs.status != JQS_FW_STATUS_STAGED)
		return;

#if IS_ENABLED(CONFIG_IPU_DEBUG)
	if (bus->jqs.status_min >= JQS_FW_STATUS_STAGED)
		return;
#endif

	dev_dbg(bus->parent_dev, "%s: unstaging firmware\n", __func__);

	ipu_core_free_shared_memory(bus, bus->jqs.fw_shared_buffer);
	bus->jqs.status = JQS_FW_STATUS_REQUESTED;
}

static int ipu_core_jqs_power_enable(struct paintbox_bus *bus,
		dma_addr_t boot_ab_paddr, dma_addr_t smem_ab_paddr,
		bool jqs_cold_boot)
{
	/* If the PCIe link is down then we are not ready */
	if (!ipu_core_is_ready(bus)) {
		dev_err(bus->parent_dev,
				"%s: unable to enable JQS, hw not ready\n",
				__func__);
		return -ENETDOWN;
	}

	/* The Airbrush IPU needs to be put in reset before turning on the
	 * I/O block.
	 */
	ipu_core_writel(bus, SOFT_RESET_IPU_MASK, IPU_CSR_AON_OFFSET +
			SOFT_RESET);

	ipu_core_writel(bus, JQS_CACHE_ENABLE_I_CACHE_MASK |
			JQS_CACHE_ENABLE_D_CACHE_MASK,
			IPU_CSR_AON_OFFSET + JQS_CACHE_ENABLE);

	ipu_core_writel(bus, (uint32_t)boot_ab_paddr, IPU_CSR_AON_OFFSET +
			JQS_BOOT_ADDR);

	/* If the I/O block is already powered then skip the pre-power, power
	 * on sequence.
	 */
	if (ipu_core_readq(bus, IPU_CSR_AON_OFFSET + IO_POWER_ON_N) != 0) {
		/* Pre-power the I/O block and then enable power */
		ipu_core_writeq(bus, IO_POWER_ON_N_MAIN_MASK,
				IPU_CSR_AON_OFFSET + IO_POWER_ON_N);
		ipu_core_writeq(bus, 0, IPU_CSR_AON_OFFSET + IO_POWER_ON_N);

		udelay(IO_POWER_RAMP_TIME);
	}

	/* We need to run the clock to the I/O block while it is being powered
	 * on briefly so that all the synchronizers clock through their data and
	 * all the Xs (or random values in the real HW) clear. Then we need to
	 * turn the clock back off so that we can meet timing on the RAM SD pin
	 * -- the setup & hold on the RAM's SD pin is significantly longer than
	 * 1 clock cycle.
	 */
	ipu_core_writel(bus, IPU_IO_SWITCHED_CLK_EN_VAL_MASK,
			IPU_CSR_AON_OFFSET + IPU_IO_SWITCHED_CLK_EN);
	ipu_core_writel(bus, 0, IPU_CSR_AON_OFFSET + IPU_IO_SWITCHED_CLK_EN);

	/* Power on RAMs for I/O block */
	ipu_core_writel(bus, 0, IPU_CSR_AON_OFFSET + IO_RAM_ON_N);
	udelay(RAM_POWER_RAIL_RAMP_TIME);

	/* Turn on clocks to I/O block */
	ipu_core_writel(bus, IPU_IO_SWITCHED_CLK_EN_VAL_MASK,
			IPU_CSR_AON_OFFSET + IPU_IO_SWITCHED_CLK_EN);

	ipu_bus_frc_clock_ungate(bus);

	/* Turn off isolation for I/O block */
	ipu_core_writel(bus, 0, IPU_CSR_AON_OFFSET + IO_ISO_ON);

	/* Take the IPU out of reset. */
	ipu_core_writel(bus, 0, IPU_CSR_AON_OFFSET + SOFT_RESET);

	ipu_core_writel(bus, (uint32_t)smem_ab_paddr, IPU_CSR_JQS_OFFSET +
			SYS_JQS_GPR_TRANSPORT);

	ipu_core_writel(bus, jqs_cold_boot ?
			JQS_BOOT_MODE_COLD : JQS_BOOT_MODE_WARM,
			IPU_CSR_JQS_OFFSET + SYS_JQS_GPR_BOOT_MODE);

	/* Enable the JQS */
	ipu_core_writel(bus, JQS_CONTROL_CORE_FETCH_EN_MASK,
			IPU_CSR_AON_OFFSET + JQS_CONTROL);

	/* Enable automatic idle clock gating for MMU, BIF, SSP, and DMA */
	ipu_core_writel(bus, 0, IPU_CSR_AON_OFFSET + CLK_GATE_CONTROL);

	return 0;
}

static void ipu_core_jqs_power_disable(struct paintbox_bus *bus)
{
	/* If the PCIe link is down then there is nothing to be done. */
	if (!ipu_core_is_ready(bus))
		return;

	/* Disable automatic idle clock gating for MMU, BIF, SSP, and DMA */
	ipu_core_writel(bus, CLK_GATE_CONTROL_MMU_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_BIF_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_DMA_IDLE_GATE_DIS_MASK,
			IPU_CSR_AON_OFFSET + CLK_GATE_CONTROL);

	ipu_core_writel(bus, 0, IPU_CSR_AON_OFFSET + JQS_CONTROL);

	/* Turn on isolation for I/O block */
	ipu_core_writel(bus, IO_ISO_ON_VAL_MASK, IPU_CSR_AON_OFFSET +
		IO_ISO_ON);

	/* Turn off clocks to I/O block */
	ipu_core_writel(bus, 0, IPU_CSR_AON_OFFSET + IPU_IO_SWITCHED_CLK_EN);

	/* Power off RAMs for I/O block */
	ipu_core_writel(bus, IO_RAM_ON_N_VAL_MASK, IPU_CSR_AON_OFFSET +
			IO_RAM_ON_N);

	/* Need to briefly turn on the clocks to the I/O block to propagate the
	 * RAM SD pin change into the RAM, then need to turn the clocks off
	 * again, since the I/O block is being turned off.
	 */
	ipu_core_writel(bus, IPU_IO_SWITCHED_CLK_EN_VAL_MASK,
			IPU_CSR_AON_OFFSET + IPU_IO_SWITCHED_CLK_EN);
	ipu_core_writel(bus, 0, IPU_CSR_AON_OFFSET +
			IPU_IO_SWITCHED_CLK_EN);

	/* Note that due to a bug in silicon the I/O block power needs to be
	 * left on.  b/124389401
	 */
}

/* The caller to this function must hold bus->jqs.lock */
static int ipu_core_jqs_start_firmware(struct paintbox_bus *bus,
	bool cold_boot)
{
	int ret;

	if ((!ipu_core_is_ready(bus)) || (!ipu_core_jqs_is_clock_ready(bus))) {
		dev_err(bus->parent_dev,
				"%s: unable to start JQS firmware, hardware not ready\n",
				__func__);
		return -ECONNREFUSED;
	}

	if (cold_boot) {
		/* For warm boots, the msg transports have already been
		 * allocated and initialized. The JQS just needs to be
		 * notified that the DRAM memory is valid. Cold boots
		 * required that msg transports need to be allocated
		 * and initialized. The JQS is also notified it must
		 * initialize everything.
		 */
		ret = ipu_core_jqs_msg_transport_init(bus);
		if (ret < 0)
			return ret;

		ret = ipu_core_jqs_msg_transport_alloc_kernel_queue(bus);
		if (ret < 0)
			goto err_shutdown_transport;
	}

	ret = ipu_core_jqs_power_enable(bus,
			bus->jqs.fw_shared_buffer->jqs_paddr,
			bus->jqs_msg_transport->shared_buf->jqs_paddr,
			cold_boot);
	if (ret < 0)
		goto err_free_kernel_queue;

	atomic_or(IPU_STATE_JQS_READY, &bus->state);

	ret = ipu_core_jqs_send_clock_rate(bus, bus->jqs.clock_rate_hz);
	if (ret < 0)
		goto err_disable_jqs;

	ret = ipu_core_jqs_send_set_log_info(bus);
	if (ret < 0)
		goto err_disable_jqs;

	bus->jqs.status = JQS_FW_STATUS_RUNNING;

	/* Notify paintbox devices that the firmware is up */
	ipu_core_notify_firmware_up(bus);

	return 0;

err_disable_jqs:
	ipu_core_jqs_power_disable(bus);
err_free_kernel_queue:
	ipu_core_jqs_msg_transport_free_kernel_queue(bus, ret);
err_shutdown_transport:
	ipu_core_jqs_msg_transport_shutdown(bus);

	return ret;
}

static inline int ipu_core_jqs_start_firmware_cold_boot(
		struct paintbox_bus *bus)
{
	return ipu_core_jqs_start_firmware(bus, true);
}

static inline int ipu_core_jqs_start_firmware_warm_boot(
		struct paintbox_bus *bus)
{
	return ipu_core_jqs_start_firmware(bus, false);
}

/* The caller to this function must hold bus->jqs.lock */
int ipu_core_jqs_enable_firmware(struct paintbox_bus *bus)
{
	int ret;

	/* Firmware status will be set to INIT at boot or if the driver is
	 * unloaded and reloaded (likely due to a PCIe link change).
	 */
	if (bus->jqs.status == JQS_FW_STATUS_INIT) {
		ret = ipu_core_jqs_load_firmware(bus);
		if (ret < 0)
			return ret;
	}

	/* If the firmware is in the requested state then stage it to DRAM.
	 * Firmware status will return this state whenever Airbrush transitions
	 * to the OFF state.
	 */
	if (bus->jqs.status == JQS_FW_STATUS_REQUESTED) {
		ret = ipu_core_jqs_stage_firmware(bus);
		if (ret < 0)
			goto unload_firmware;
	}

	/* If the firmware has been staged then enable the firmware.  Firmware
	 * status will return to this state for all suspend and sleep states
	 * with the exception of OFF.
	 */
	if (bus->jqs.status == JQS_FW_STATUS_STAGED) {
		ret = ipu_core_jqs_start_firmware_cold_boot(bus);
		if (ret < 0)
			goto unstage_firmware;
	}

	/* If the firmware has been suspended then reenable the firmware
	 */
	if (bus->jqs.status == JQS_FW_STATUS_SUSPENDED) {
		ret = ipu_core_jqs_start_firmware_warm_boot(bus);
		if (ret < 0)
			goto unstage_firmware;
	}

	return 0;

unstage_firmware:
	ipu_core_jqs_unstage_firmware(bus);
unload_firmware:
	ipu_core_jqs_unload_firmware(bus);

	return ret;
}

/* The caller to this function must hold bus->jqs.lock */
void ipu_core_jqs_suspend_firmware(struct paintbox_bus *bus)
{
	int shutdown_timeout;
	int ret;

	if (!ipu_core_jqs_is_ready(bus))
		return;

	ret = ipu_core_jqs_send_shutdown_mode(bus,
			JQS_SHUTDOWN_MODE_FOR_RESUME);

	if (ret < 0) {
		ipu_core_jqs_shutdown_firmware(bus);
		return;
	}

	/*  Wait for the JQS to set the shutdown GPR indicating the cache has
	 *  been flushed and the hardware shutdown
	 */
	shutdown_timeout = 0;
	while ((ipu_core_readl(bus, IPU_CSR_JQS_OFFSET + JQS_SYS_GPR_SHUTDOWN)
			!= JQS_SHUTDOWN_COMPLETE) &&
			(shutdown_timeout < JQS_SHUTDOWN_TIMEOUT_LIMIT_US)) {

		usleep_range(JQS_SHUTDOWN_TIMEOUT_INTERVAL_US_LOWER,
				JQS_SHUTDOWN_TIMEOUT_INTERVAL_US_UPPER);

		shutdown_timeout += JQS_SHUTDOWN_TIMEOUT_INTERVAL_US;
	}

	ipu_core_jqs_disable_firmware_suspended(bus);

	if (shutdown_timeout <= JQS_SHUTDOWN_TIMEOUT_LIMIT_US) {
		bus->jqs.status = JQS_FW_STATUS_SUSPENDED;
		ipu_core_notify_firmware_suspended(bus);
	} else {
		ipu_core_jqs_shutdown_firmware(bus);
	}
}

/* The caller to this function must hold bus->jqs.lock */
void ipu_core_jqs_shutdown_firmware(struct paintbox_bus *bus)
{
	/* JQS reset will be complete after this function has returned and the
	 * bus->jqs.lock has been released. At that point the client and request
	 * a cold boot of the JQS.
	 */
	bus->jqs.reset_in_progress = false;

	if (bus->jqs.status == JQS_FW_STATUS_REQUESTED) {
		ipu_core_notify_firmware_down(bus);
		return;
	}

	if (ipu_core_jqs_is_ready(bus))
		ipu_core_jqs_send_shutdown_mode(bus, JQS_SHUTDOWN_MODE_HARD);

	ipu_core_jqs_disable_firmware_fatal_error(bus);
	ipu_core_jqs_unstage_firmware(bus);
}

/* The caller to this function must hold bus->jqs.lock */
static void ipu_core_jqs_disable_firmware(struct paintbox_bus *bus)
{
	if ((bus->jqs.status != JQS_FW_STATUS_RUNNING) &&
			(bus->jqs.status != JQS_FW_STATUS_SUSPENDED))
		return;

#if IS_ENABLED(CONFIG_IPU_DEBUG)
	if (bus->jqs.status_min >= JQS_FW_STATUS_RUNNING)
		return;
#endif

	dev_dbg(bus->parent_dev, "%s: disabling firmware\n", __func__);

	ipu_core_jqs_power_disable(bus);

	atomic_andnot(IPU_STATE_JQS_READY, &bus->state);
	bus->jqs.status = JQS_FW_STATUS_STAGED;
}

void ipu_core_jqs_disable_firmware_requested(struct paintbox_bus *bus)
{
	/* Notify paintbox devices that the firmware is down.  The IPU client
	 * will free any application queues and unblock any waiting threads.
	 */
	ipu_core_notify_firmware_down(bus);

	/* Firmware disable requests initiated by the AP are reported as aborts
	 * on any queue that is still active when the disable request is made.
	 */
	ipu_core_jqs_msg_transport_free_kernel_queue(bus, -ECONNABORTED);
	ipu_core_jqs_msg_transport_shutdown(bus);
	ipu_core_jqs_disable_firmware(bus);
}

void ipu_core_jqs_disable_firmware_suspended(struct paintbox_bus *bus)
{
	ipu_core_jqs_msg_transport_complete_kernel_queue(bus, -ENETRESET);
	ipu_core_jqs_disable_firmware(bus);
}

void ipu_core_jqs_disable_firmware_fatal_error(struct paintbox_bus *bus)
{
	/* Notify paintbox devices that the firmware is down.  The IPU client
	 * will free any application queues and unblock any waiting threads.
	 */
	ipu_core_notify_firmware_down(bus);

	/* Firmware disable requests initiated by fatal errors are reported
	 * as hard resets.
	 */
	ipu_core_jqs_msg_transport_free_kernel_queue(bus, -ECONNRESET);
	ipu_core_jqs_msg_transport_shutdown(bus);
	ipu_core_jqs_disable_firmware(bus);
}

int ipu_core_jqs_start(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;
	int ret;

	dev_dbg(bus->parent_dev, "%s: runtime request to power up JQS\n",
			__func__);

	mutex_lock(&bus->jqs.lock);

	/* Check to make sure there are IPU clients before enabling the
	 * firmware.  The start() hook can be invoked as part of device power
	 * management, as well as, runtime power management.
	 */
	if (bus->jqs.client_count == 0) {
		mutex_unlock(&bus->jqs.lock);
		return -ENETDOWN;
	}

	if (bus->jqs.reset_in_progress) {
		mutex_unlock(&bus->jqs.lock);
		dev_err(bus->parent_dev, "%s: reset in progress\n", __func__);
		return -ENETDOWN;
	}

	if (WARN_ON(!ipu_core_is_ready(bus) ||
			!ipu_core_jqs_is_clock_ready(bus))) {
		mutex_unlock(&bus->jqs.lock);
		return -ENETDOWN;
	}

	ret = ipu_core_jqs_enable_firmware(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

int ipu_core_jqs_power_down(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	dev_dbg(bus->parent_dev, "%s: runtime request to power down JQS\n",
			__func__);

	mutex_lock(&bus->jqs.lock);

	/* Check to make sure there are no IPU clients before disabling the
	 * firmware.
	 */
	if (bus->jqs.client_count > 0) {
		dev_err(bus->parent_dev,
			"%s: tried to stop jqs while clients are active\n",
			__func__);
		mutex_unlock(&bus->jqs.lock);
		return -EINPROGRESS;
	}

	if (!bus->jqs.reset_in_progress)
		ipu_core_jqs_disable_firmware_requested(bus);

	mutex_unlock(&bus->jqs.lock);

	return 0;
}

/* The caller to this function must hold bus->jqs.lock */
void ipu_core_jqs_resume_firmware(struct paintbox_bus *bus,
		uint64_t ipu_clock_rate_hz)
{
	int ret;

	bus->jqs.clock_rate_hz = ipu_clock_rate_hz;

	if (bus->jqs.client_count == 0 || bus->jqs.reset_in_progress ||
			!ipu_core_is_ready(bus) ||
			!ipu_core_jqs_is_clock_ready(bus))
		return;

	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_clock_rate(bus, ipu_clock_rate_hz);
	else
		ret = ipu_core_jqs_enable_firmware(bus);

	if (ret < 0)
		dev_err(bus->parent_dev,
			"%s: failed to resume firmware, err %d\n",
			__func__, ret);
}

int ipu_core_jqs_init(struct paintbox_bus *bus)
{
	int ret;

	mutex_init(&bus->jqs.lock);

	/* Try to load the pre-load the firmware if it is available. */
	ret = ipu_core_jqs_load_firmware(bus);
	if (ret < 0)
		dev_warn(bus->parent_dev,
				"%s: unable to preload JQS firmware, ret %d\n",
				__func__, ret);

	/* TODO(b/116190655):  Once the FW download SPI issue is resolved and we
	 * can start DRAM at PCIe probe we should try to stage the JQS FW to
	 * Airbrush DRAM at IPU driver probe.
	 */

	bus->jqs.log_level = JQS_LOG_LEVEL_DEF;
	bus->jqs.log_trigger_level = JQS_LOG_TRIGGER_LEVEL_DEF;
	bus->jqs.log_sink_mask = JQS_LOG_SINK_DEF;
	bus->jqs.uart_baud = JQS_LOG_UART_BAUD_DEF;

#if IS_ENABLED(CONFIG_IPU_DEBUG)
	bus->jqs.status_min = JQS_FW_STATUS_INIT;
#endif

	ret = ipu_core_jqs_create_sysfs(bus->parent_dev);
	if (ret < 0) {
		dev_err(bus->parent_dev,
				"%s: unable to create sysfs files for IPU JQS, ret %d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

void ipu_core_jqs_remove(struct paintbox_bus *bus)
{
	mutex_lock(&bus->jqs.lock);

	ipu_core_jqs_disable_firmware_requested(bus);
	ipu_core_jqs_unstage_firmware(bus);
	ipu_core_jqs_unload_firmware(bus);

	mutex_unlock(&bus->jqs.lock);

	ipu_core_jqs_remove_sysfs(bus->parent_dev);

	mutex_destroy(&bus->jqs.lock);
}

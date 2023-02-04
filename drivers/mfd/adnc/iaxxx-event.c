/*
 * iaxxx-event.c -- IAxxx events
 *
 * Copyright 2017 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/mm.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include "iaxxx.h"

#define BIT_FNAME(N, T)  N ## _ ## T
#define GET_BITS(R, N)	(((R) & BIT_FNAME(N, MASK)) >> BIT_FNAME(N, POS))

/*****************************************************************************
 * iaxxx_core_evt_is_valid_src_id()
 * @brief validate the plugin event scr id
 *
 * @id               event scr id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_evt_is_valid_src_id(uint32_t src_id)
{
	bool ret = true;

	if (src_id > (IAXXX_EVT_MGMT_EVT_SUB_SRC_ID_MASK
					>> IAXXX_EVT_MGMT_EVT_SUB_SRC_ID_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_is_valid_src_id);

/*****************************************************************************
 * iaxxx_core_evt_is_valid_dst_id()
 * @brief validate the plugin event dest id
 *
 * @id              Plugin  event dst id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_evt_is_valid_dst_id(uint32_t dst_id)
{
	bool ret = true;

	if (dst_id > (IAXXX_EVT_MGMT_EVT_SUB_DST_ID_MASK
					>> IAXXX_EVT_MGMT_EVT_SUB_DST_ID_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_is_valid_dst_id);

/*****************************************************************************
 * iaxxx_core_evt_is_valid_event_id()
 * @brief validate the plugin event id
 *
 * @id              Plugin  event event id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_evt_is_valid_event_id(uint32_t event_id)
{
	bool ret = true;

	if (event_id > (IAXXX_EVT_MGMT_EVT_ID_REG_MASK
					>> IAXXX_EVT_MGMT_EVT_ID_REG_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_is_valid_event_id);

/*****************************************************************************
 * iaxxx_core_evt_is_valid_dst_opaque()
 * @brief validate the plugin dst opaque
 *
 * @id              Plugin  event dst opaque
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_evt_is_valid_dst_opaque(uint32_t dst_opaque)
{
	bool ret = true;

	if (dst_opaque > (IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_REG_MASK
				>> IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_REG_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_is_valid_dst_opaque);

/*****************************************************************************
 * iaxxx_core_evt_subscribe()
 * @brief Subscribe to an event
 *
 * @src_id     -   System Id of event source
 * @event_id   -   Event Id
 * @dst_id     -   System Id of event destination
 * @ds_opaque  -   Information sought by destination task when even occurs.
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_evt_subscribe(struct device *dev, uint16_t src_id,
			uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque)
{
	int ret = -EINVAL;
	int status;
	uint32_t sys_id;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_dbg(dev,
		"%s() src_id : 0x%x dst_id: 0x%x\n", __func__, src_id, dst_id);

	if (src_id == IAXXX_SYSID_INVALID || dst_id == IAXXX_SYSID_INVALID) {
		dev_err(dev, "Invalid System Ids %s()\n", __func__);
		return ret;
	}

	/*
	 * Update all event subscription registers
	 * Event ID, IDS of source and destination, destination opaque
	 */
	mutex_lock(&priv->event_lock);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_ID_ADDR, event_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto evt_err;
	}
	sys_id = ((dst_id << 16) | src_id);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_ADDR, sys_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto evt_err;
	}
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_ADDR,
			dst_opaque);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto evt_err;
	}
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
		(1 << IAXXX_EVT_MGMT_EVT_SUB_REQ_POS),
		IAXXX_EVT_MGMT_EVT_SUB_REQ_MASK);
	if (ret) {
		dev_err(dev, "Update bit failed %s()\n", __func__);
		goto evt_err;
	}
	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(dev, "Update blk failed %s()\n", __func__);
evt_err:
	mutex_unlock(&priv->event_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_subscribe);

/*****************************************************************************
 * iaxxx_core_evt_unsubscribe()
 * @brief UnSubscribe to an event
 *
 * @src_id     -   System Id of event source
 * @event_id   -   Event Id
 * @dst_id     -   System Id of event destination
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_evt_unsubscribe(struct device *dev, uint16_t src_id,
			uint16_t event_id, uint16_t dst_id)
{
	int ret = -EINVAL;
	uint32_t status;
	uint32_t sys_id;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s()\n", __func__);

	if (src_id == IAXXX_SYSID_INVALID || dst_id == IAXXX_SYSID_INVALID) {
		dev_err(dev, "Invalid System Ids %s()\n", __func__);
		return ret;
	}
	/*
	 * Update all event subscription registers
	 * Event ID, Subsystem IDS of source and destination, destination
	 *  opaque
	 */
	mutex_lock(&priv->event_lock);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_ID_ADDR, event_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto err;
	}
	sys_id = ((dst_id << 16) | src_id);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_ADDR, sys_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto err;
	}
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
		(1 << IAXXX_EVT_MGMT_EVT_UNSUB_REQ_POS),
		IAXXX_EVT_MGMT_EVT_UNSUB_REQ_MASK);
	if (ret) {
		dev_err(dev, "Update bit failed %s()\n", __func__);
		goto err;
	}
	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(dev, "Update blk failed %s()\n", __func__);

err:
	mutex_unlock(&priv->event_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_unsubscribe);

/*****************************************************************************
 * @brief Fetches next event subscription entry from the last read position
 *
 * @param[out] src_id     -   System Id of event source
 * @param[out] evt_id     -   Event Id
 * @param[out] dst_id     -   System Id of event destination
 * @param[out] dst_opaque -   Destination opaque data
 *
 * @ret 0 on success, -EINVAL in case of error
 *****************************************************************************/
int iaxxx_core_evt_read_subscription(struct device *dev,
					uint16_t *src_id,
					uint16_t *evt_id,
					uint16_t *dst_id,
					uint32_t *dst_opaque)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int ret;
	uint32_t value = 0;
	uint32_t status;

	mutex_lock(&priv->event_lock);

	/* 1. Set the SUB_READ_REQ bit in EVT register to read subscription. */
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
				IAXXX_EVT_MGMT_EVT_SUB_READ_REQ_MASK,
				(1 << IAXXX_EVT_MGMT_EVT_SUB_READ_REQ_POS));
	if (ret) {
		dev_err(dev,
	"Setting the SUB_RET_REQ bit in EVT register failed %s()\n", __func__);
		goto reg_err;
	}

	/*
	 * 2. Set the REQ bit in the SYS_BLK_UPDATE register.
	 * 3. Wait for the REQ bit to clear. The device will also clear the
	 *    SUB_READ_REQ bit in the EVT register automatically.
	 * 4. Check the RES field in the SYS_BLK_UPDATE register to make sure
	 *    that the operation succeeded (content is 0x0).
	 */
	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto reg_err;
	}

	/*
	 * 5. Read registers EVT_ID, EVT_SUB(source and destination Ids)
	 *    and EVT_DST_OPAQUE
	 */
	ret = regmap_read(priv->regmap,
				IAXXX_EVT_MGMT_EVT_DST_OPAQUE_ADDR, &value);
	if (ret) {
		dev_err(dev,
			"Failed to read IAXXX_EVT_MGMT_EVT_DST_OPAQUE_ADDR\n");
		goto reg_err;
	}
	*dst_opaque = value;

	ret = regmap_read(priv->regmap,
				IAXXX_EVT_MGMT_EVT_ID_ADDR, &value);
	if (ret) {
		dev_err(dev, "Failed to read IAXXX_EVT_MGMT_EVT_ID_ADDR\n");
		goto reg_err;
	}
	*evt_id = (uint16_t)value;

	ret = regmap_read(priv->regmap,
				IAXXX_EVT_MGMT_EVT_SUB_ADDR, &value);
	if (ret) {
		dev_err(dev, "Failed to read IAXXX_EVT_MGMT_EVT_SUB_ADDR\n");
		goto reg_err;
	}
	*src_id = (uint16_t)((value & IAXXX_EVT_MGMT_EVT_SUB_SRC_ID_MASK)
					>> IAXXX_EVT_MGMT_EVT_SUB_SRC_ID_POS);
	*dst_id = (uint16_t)((value & IAXXX_EVT_MGMT_EVT_SUB_DST_ID_MASK)
					>> IAXXX_EVT_MGMT_EVT_SUB_DST_ID_POS);

reg_err:
	mutex_unlock(&priv->event_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_read_subscription);

/*****************************************************************************
 * @brief Retrieve an event notification
 *
 *  @param[out] *src_id       pointer to uint16_t for reporting SystemId of
 *                            event source
 *  @param[out] *evt_dd       pointer to uint16_t for reporting Id of event
 *  @param[out] *src_opaque   pointer to the first parameter of event
 *  @param[out] *dst_opaque   pointer to the second parameter of event.
 *                            This will be destOpaque in case if event is
 *                            subscribed with valid destOpaque otherwise
 *                            it will be used as second parameter.
 *
 *  @return 0 if successful, error number in case of error
 *****************************************************************************/
int iaxxx_core_evt_retrieve_notification(struct device *dev,
						uint16_t *src_id,
						uint16_t *evt_id,
						uint32_t *src_opaque,
						uint32_t *dst_opaque)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int ret = 0;
	uint32_t value = 0;
	uint32_t status;

	mutex_lock(&priv->event_lock);
	/*
	 * 1. Read the number of pending events N from the EVENT_COUNT register
	 *    Repeat the following steps N times(exit if N = 0)
	 */
	ret = regmap_read(priv->regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR, &value);
	if (ret) {
		dev_err(dev, "Getting number of event notifications failed\n");
		goto reg_err;
	}
	if (value == 0) {
		*src_id      = (uint16_t)IAXXX_SYSID_INVALID;
		*evt_id      = 0;
		*src_opaque  = 0;
		*dst_opaque  = 0;
		goto reg_err;
	}

	/*
	 * 2. Set the NOT(notification) bit in the EVT_NEXT_REQ register.
	 */
	ret = regmap_write(priv->regmap,
				IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
				IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK);
	if (ret) {
		dev_err(dev,
			"Writing request to retrieve notification failed\n");
		goto reg_err;
	}

	/*
	 * 3. Set the REQ bit in the SYS_BLK_UPDATE register.
	 * 4. WARNING: The Host should not set the UPDATE_COMPLETE_ENABLE,
	 *    as that will result in a new Event being generated for the block
	 *    completion.
	 * 5. Wait for the REQ bit to clear. The device will also clear the
	 *    NOT(notification) bit in the EVT_NEXT_REQ register automatically.
	 * 6. Check the RES field in the SYS_BLK_UPDATE register to make sure
	 *    that the operation succeeded (content is 0x0).
	 */
	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret) {
		dev_err(dev, "%s() Update blk failed\n", __func__);
		goto reg_err;
	}

	/*
	 * 7. Read the EVENT_SRC_INFO, EVT_SRC_OPAQUE, and EVT_DST_OPAQUE
	 *    registers.
	 */
	ret = regmap_read(priv->regmap,
				IAXXX_EVT_MGMT_EVT_SRC_INFO_ADDR, &value);
	if (ret) {
		dev_err(dev, "Getting source information failed\n");
		goto reg_err;
	}

	*src_id = (uint16_t)((value & IAXXX_EVT_MGMT_EVT_SRC_INFO_SYS_ID_MASK)
				>> IAXXX_EVT_MGMT_EVT_SRC_INFO_SYS_ID_POS);
	*evt_id = (uint16_t)((value & IAXXX_EVT_MGMT_EVT_SRC_INFO_EVT_ID_MASK)
				>> IAXXX_EVT_MGMT_EVT_SRC_INFO_EVT_ID_POS);

	ret = regmap_read(priv->regmap,
				IAXXX_EVT_MGMT_EVT_SRC_OPAQUE_ADDR, &value);
	if (ret) {
		dev_err(dev, "Getting source opaque failed\n");
		goto reg_err;
	}
	*src_opaque = value;

	ret = regmap_read(priv->regmap,
				IAXXX_EVT_MGMT_EVT_DST_OPAQUE_ADDR, &value);
	if (ret) {
		dev_err(dev, "Getting destination opaque failed\n");
		goto reg_err;
	}
	*dst_opaque = value;

reg_err:
	mutex_unlock(&priv->event_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_retrieve_notification);

/*****************************************************************************
 *  @brief Reset index for retrieving subscription entries
 *
 *  @param   void
 *
 *  @return 0 if successful, error number in case of error
 *****************************************************************************/
int iaxxx_core_evt_reset_read_index(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int ret;
	uint32_t status;

	mutex_lock(&priv->event_lock);
	/* Set the RESET_RD_IDX  bit to read subscription */
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
				IAXXX_EVT_MGMT_EVT_RESET_RD_IDX_MASK,
				(1 << IAXXX_EVT_MGMT_EVT_RESET_RD_IDX_POS));
	if (ret) {
		dev_err(dev,
		"%s() Setting the RESET_RD_IDX bit in EVT register failed\n",
								__func__);
		goto reg_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(dev, "%s() Update blk failed\n", __func__);

reg_err:
	mutex_unlock(&priv->event_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_reset_read_index);

/*****************************************************************************
 * iaxxx_core_evt_trigger()
 *  @brief Trigger an event. This may be most useful when debugging the system,
 *        but can also be used to trigger simultaneous behavior in entities
 *        which have subscribed, or to simply provide notifications regarding
 *        host status:
 *
 *  @param[in] src_id        SystemId of event source
 *  @param[in] evt_id        Id of event
 *  @param[in] src_opaque    Source opaque to pass with event notification
 *
 *  @return 0 if successful, error number in case of error
 ****************************************************************************/
int iaxxx_core_evt_trigger(struct device *dev,
			uint16_t src_id, uint16_t evt_id, uint32_t src_opaque)
{
	int ret = -EINVAL;
	int status;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s() src_id=%u, evt_id=%u, src_opaque=%u\n",
				__func__, src_id, evt_id, src_opaque);

	if (src_id == IAXXX_SYSID_INVALID) {
		dev_err(dev, "Invalid System Ids %s()\n", __func__);
		return ret;
	}

	mutex_lock(&priv->event_lock);
	/*
	 * 1. Set the System ID(src Id and evt Id) in the field of
	 *    EVT_SRC_INFO register.
	 */
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SRC_INFO_ADDR,
			(src_id << IAXXX_EVT_MGMT_EVT_SRC_INFO_SYS_ID_POS)
			| (evt_id << IAXXX_EVT_MGMT_EVT_SRC_INFO_EVT_ID_POS));
	if (ret) {
		dev_err(dev, "Writing source information failed %s()\n",
								__func__);
		goto reg_err;
	}

	/*
	 * 2. Set the source opaque data by writing to
	 *    the EVT_SRC_OPAQUE register.
	 */
	ret = regmap_write(priv->regmap,
			IAXXX_EVT_MGMT_EVT_SRC_OPAQUE_ADDR, src_opaque);
	if (ret) {
		dev_err(dev, "Writing source opaque failed %s()\n", __func__);
		goto reg_err;
	}

	/* 3. Set the TRIG_REQ bit (and only it) in EVT register. */
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
				IAXXX_EVT_MGMT_EVT_TRIG_REQ_MASK,
				(1 << IAXXX_EVT_MGMT_EVT_TRIG_REQ_POS));
	if (ret) {
		dev_err(dev,
		    "Setting the TRIG_REQ bit in EVT register failed %s()\n",
		    __func__);
		goto reg_err;
	}

	/*
	 * 4. Set the REQ bit in the SYS_BLK_UPDATE register
	 *
	 * 5. Wait for the REQ bit to clear.
	 *    The device will also clear the EVENT_SUB_REQ bit automatically.
	 *
	 * 6. Check the RES field in the SYS_BLK_UPDATE register
	 *    to make sure that the operation succeeded (content is 0x0)
	 */

	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(dev, "Update blk failed %s()\n", __func__);

reg_err:
	mutex_unlock(&priv->event_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_trigger);

/*****************************************************************************
 * iaxxx_core_retrieve_event()
 * @brief Retrieve an event notification
 *
 * @event_id	-	Event Id
 * @data	-	Event data
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_retrieve_event(struct device *dev, uint16_t *event_id,
		uint32_t *data)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int r_index = priv->event_queue->r_index;

	if (!priv)
		return ret;

	dev_dbg(dev, "%s()\n", __func__);

	mutex_lock(&priv->event_queue_lock);
	r_index++;
	/* Check if there are no events */
	if (r_index == (priv->event_queue->w_index + 1)) {
		dev_err(dev, "%s Buffer underflow\n", __func__);
		mutex_unlock(&priv->event_queue_lock);
		return ret;
	}
	if (r_index == IAXXX_MAX_EVENTS)
		r_index = 0;

	priv->event_queue->r_index = r_index;
	*event_id = priv->event_queue->event_info[r_index].event_id;
	*data = priv->event_queue->event_info[r_index].data;
	pr_debug("%s() event Id %d, data %d read index %d\n", __func__,
			*event_id, *data, r_index);
	mutex_unlock(&priv->event_queue_lock);
	return 0;
}
EXPORT_SYMBOL(iaxxx_core_retrieve_event);

/*===========================================================================
 * Event Notification
 *===========================================================================
 */

static int iaxxx_next_event_request(struct device *dev,
		struct regmap *regmap,
		struct iaxxx_event *ev)
{
	int rc;
	uint32_t status, data[3];
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	mutex_lock(&priv->event_lock);
	/* Set the next event notification request */
	rc = regmap_update_bits(regmap,
			IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
			IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK,
			0x1 << IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_POS);
	if (rc) {
		dev_err(dev, "Failed to set EVT_NEXT_REQ, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_send_update_block_request_with_options(
			dev, IAXXX_BLOCK_0, IAXXX_HOST_0, regmap, 0,
			UPDATE_BLOCK_NO_OPTIONS,
			&status);
	if (rc) {
		dev_err(dev, "EVT_NEXT_REQ failed, rc = %d\n", rc);

		/* Clear the request bit */
		regmap_update_bits(regmap,
				IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
				IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK, 0);

		goto out;
	}

	/* The EVT_NEXT_REQ bit should have been cleared by firmware */
	rc = regmap_read(regmap, IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
			&status);
	if (rc) {
		dev_err(dev, "Failed to read EVT_NEXT_REQ, rc = %d\n", rc);
		goto out;
	}

	WARN_ON(status & IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK);
	rc = regmap_bulk_read(regmap,
			IAXXX_EVT_MGMT_EVT_SRC_INFO_ADDR, data,
			ARRAY_SIZE(data));
	if (rc) {
		dev_err(dev, "Failed to read EVENT_INFO, rc = %d\n", rc);
		goto out;
	}

	ev->event_src = GET_BITS(data[0],
			IAXXX_EVT_MGMT_EVT_SRC_INFO_SYS_ID);
	ev->event_id  = GET_BITS(data[0],
			IAXXX_EVT_MGMT_EVT_SRC_INFO_EVT_ID);
	ev->src_opaque = data[1];
	ev->dst_opaque = data[2];

out:
	mutex_unlock(&priv->event_lock);
	return rc;
}

/**
 * iaxxx_get_event_work - Work function to read events from the event queue
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * This work function is scheduled by the ISR when any data is found in the
 * event queue.
 *
 * This function reads the available events from the queue and passes them
 * along to the event manager.
 */
static void iaxxx_get_event_work(struct work_struct *work)
{
	int rc;
	uint32_t count;
	struct iaxxx_event event;
	struct iaxxx_priv *priv = container_of(work, struct iaxxx_priv,
							event_work_struct);
	struct device *dev = priv->dev;
	struct regmap *regmap;
	int retry_cnt = 5;
	uint32_t status;
	int mode;

	rc = iaxxx_wait_dev_resume(dev);
	if (rc) {
		if (rc == -EAGAIN)
			queue_work(priv->event_workq, &priv->event_work_struct);
		else if (rc == -ETIME)
			dev_err(dev, "%s: wait resume fail\n", __func__);
		return;
	}

retry_reading_count_reg:
	/*
	 * Since this ISR can happen anytime,
	 * choose which regmap to use to read the event
	 * based on boot state.
	 */
	regmap = !iaxxx_is_firmware_ready(priv) ? priv->regmap_no_pm :
			priv->regmap;

	/* Any events in the event queue? */
	rc = regmap_read(regmap,
			IAXXX_EVT_MGMT_EVT_COUNT_ADDR, &count);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read EVENT_COUNT, rc = %d\n", rc);
		/* Read should not fail recover the chip */
		count = 0;
	}

	if (count > 0) {
		dev_dbg(priv->dev, "%s: %d event(s) avail\n", __func__, count);
		complete_all(&priv->cmem_done);
	} else {
		/* Read SYSTEM_STATUS to ensure that device is in App Mode */
		rc = regmap_read(regmap,
					IAXXX_SRB_SYS_STATUS_ADDR, &status);

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		if (rc || (mode != SYSTEM_STATUS_MODE_APPS)) {
			dev_err(priv->dev,
				"Not in app mode CM4 might crashed, mode = %d"
				" rc = %d\n", mode, rc);
			priv->cm4_crashed = true;
		} else if (mode == SYSTEM_STATUS_MODE_APPS && retry_cnt--) {
			goto retry_reading_count_reg;
		} else {
			complete_all(&priv->cmem_done);
			return;
		}
	}

	mutex_lock(&priv->event_work_lock);

	if (priv->cm4_crashed) {
		dev_dbg(priv->dev, "CM4 crash event handler called:%d\n",
							priv->cm4_crashed);
		rc = iaxxx_fw_crash(dev, IAXXX_FW_CRASH_EVENT, IAXXX_CM4_ID);
		if (rc == -EBUSY) {
			priv->cm4_crashed = false;
		}
		goto out;
	}

	/* Read the count of available events */
	rc = regmap_read(regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
			&count);
	if (rc) {
		dev_err(dev, "Failed to read EVENT_COUNT, rc = %d\n", rc);
		goto out;
	}

	while (count) {
		rc = iaxxx_next_event_request(dev, regmap, &event);
		if (rc) {
			dev_err(dev, "Failed to read event, rc = %d\n", rc);
			goto out;
		}
		rc = iaxxx_event_handler(priv, &event);
		if (rc) {
			dev_err(dev, "Event 0x%.04X:0x%.04X not delivered\n",
					event.event_src, event.event_id);
			goto out;
		}
		/* Read the count of available events */
		rc = regmap_read(regmap,
				IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
				&count);
		if (rc) {
			dev_err(dev, "Failed to read EVENT_COUNT, rc = %d\n",
					rc);
			goto out;
		}
	}

out:
	mutex_unlock(&priv->event_work_lock);
}

/**
 * iaxxx_event_init - Initialize Event Queue
 *
 * @priv : iaxxx private data
 */
int iaxxx_event_init(struct iaxxx_priv *priv)
{
	int rc;

	priv->event_queue = kvmalloc(sizeof(struct iaxxx_evt_queue),
					GFP_KERNEL);
	if (!priv->event_queue)
		return -ENOMEM;
	priv->event_queue->r_index = -1;
	priv->event_queue->w_index = -1;
	priv->event_workq =
		alloc_workqueue("iaxxx-evnt-wq", WQ_MEM_RECLAIM, 0);
	if (!priv->event_workq) {
		pr_err("%s: failed to register event workq\n",
				__func__);
		rc = -ENOMEM;
		kvfree(priv->event_queue);
		return rc;
	}
	/* Set the work queue function as iaxxx_get_event_work() */
	INIT_WORK(&priv->event_work_struct, iaxxx_get_event_work);
	return 0;
}

/**
 * iaxxx_event_exit - Free Event Queue
 *
 * @priv : iaxxx private data
 */
void iaxxx_event_exit(struct iaxxx_priv *priv)
{
	kvfree(priv->event_queue);
	destroy_workqueue(priv->event_workq);
	priv->event_workq = NULL;
}

/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/firmware.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/compat.h>
#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <dsp/msm-cirrus-playback.h>

#define CRUS_TX_CONFIG "crus_sp_config_%s_tx.bin"
#define CRUS_RX_CONFIG "crus_sp_config_%s_rx.bin"

#define CRUS_TX_MULTI_CONFIG "crus_sp_config_%s_tx_%d.bin"
#define CRUS_RX_MULTI_CONFIG "crus_sp_config_%s_rx_%d.bin"

#define CIRRUS_RX_TOPOLOGY 0x10000CCC
#define CIRRUS_TX_TOPOLOGY 0x10001CCC

#define CRUS_AMP_FACTOR 71498
#define CRUS_PARAM_TEMP_MAX_LENGTH 384
#define CRUS_RX_GET_TEMP_BUFFER_SIZE 384
#define LEFT_OFFSET_OUT_CAL0 12
#define LEFT_OFFSET_OUT_CAL1 13
#define LEFT_OFFSET_Z 4
#define LEFT_OFFSET_TEMP 10
#define LEFT_OFFSET_R 3

#define RIGHT_OFFSET_OUT_CAL0 14
#define RIGHT_OFFSET_OUT_CAL1 15
#define RIGHT_OFFSET_Z 2
#define RIGHT_OFFSET_TEMP 10
#define RIGHT_OFFSET_R 1

#define MAGIC_MATERIAL 250

/* TODO: Consider to remove these global variables b/117298560 */
static struct device *crus_sp_device;
static atomic_t crus_sp_misc_usage_count;

static struct crus_single_data_t crus_enable;

static struct crus_sp_ioctl_header crus_sp_hdr;
static struct cirrus_cal_result_t crus_sp_cal_rslt;
static int32_t *crus_sp_get_buffer;
static int32_t crus_sp_get_buffer_size;
static atomic_t crus_sp_get_param_flag;
struct mutex crus_sp_get_param_lock;
struct mutex crus_sp_lock;
static int cirrus_sp_en;
static int cirrus_sp_usecase;
static int cirrus_fb_port_ctl;
static int cirrus_fb_load_conf_sel;
static int cirrus_fb_delta_sel;
static int cirrus_ff_chan_swap_sel;
static int cirrus_ff_chan_swap_dur;
static bool cirrus_fail_detect_en;
static int cirrus_fb_port = AFE_PORT_ID_TERTIARY_TDM_TX;
static int cirrus_ff_port = AFE_PORT_ID_TERTIARY_TDM_RX;

static int crus_sp_usecase_dt_count;
static const char *crus_sp_usecase_dt_text[MAX_TUNING_CONFIGS];

static bool swap_calibration;
static struct crus_cal_t crus_cal;
static struct cirrus_spk_component crus_spk;
static unsigned char count_config;

static bool msm_crus_is_cirrus_afe_topology(void)
{
	if (afe_get_topology(cirrus_ff_port) == CIRRUS_RX_TOPOLOGY
		&& afe_get_topology(cirrus_fb_port) == CIRRUS_TX_TOPOLOGY)
		return true;
	return false;
}

static struct afe_custom_crus_get_config_t *crus_alloc_afe_get_header(int length,
		int port, int module, int param)
{
	struct afe_custom_crus_get_config_t *config = NULL;
	int size = sizeof(struct afe_custom_crus_get_config_t);
	int index = afe_get_port_index(port);
	uint16_t payload_size = sizeof(struct param_hdr_v1) + length;

	/* Allocate memory for the message */
	config = kzalloc(size, GFP_KERNEL);
	if (!config)
		return NULL;

	/* Set header section */
	config->param.apr_hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	config->param.apr_hdr.pkt_size = size;
	config->param.apr_hdr.src_svc = APR_SVC_AFE;
	config->param.apr_hdr.src_domain = APR_DOMAIN_APPS;
	config->param.apr_hdr.src_port = 0;
	config->param.apr_hdr.dest_svc = APR_SVC_AFE;
	config->param.apr_hdr.dest_domain = APR_DOMAIN_ADSP;
	config->param.apr_hdr.dest_port = 0;
	config->param.apr_hdr.token = index;
	config->param.apr_hdr.opcode = AFE_PORT_CMD_GET_PARAM_V2;

	/* Set param section */
	config->param.port_id = (uint16_t) port;
	config->param.mem_hdr.data_payload_addr_lsw = 0;
	config->param.mem_hdr.data_payload_addr_msw = 0;
	config->param.mem_hdr.mem_map_handle = 0;
	config->param.module_id = (uint32_t) module;
	config->param.param_id = (uint32_t) param;
	/* max data size of the param_ID/module_ID combination */
	config->param.payload_size = payload_size;

	/* Set data section */
	config->param.param_hdr.module_id = (uint32_t) module;
	config->param.param_hdr.param_id = (uint32_t) param;
	config->param.param_hdr.reserved = 0; /* Must be set to 0 */
	/* actual size of the data for the module_ID/param_ID pair */
	config->param.param_hdr.param_size = length;

	return config;
}

static struct afe_custom_crus_set_config_t *crus_alloc_afe_set_header(int length,
		int port, int module, int param)
{
	struct afe_custom_crus_set_config_t *config = NULL;
	int size = sizeof(struct afe_custom_crus_set_config_t) + length;
	int index = afe_get_port_index(port);
	uint16_t payload_size = sizeof(struct param_hdr_v1) + length;

	/* Allocate memory for the message */
	config = kzalloc(size, GFP_KERNEL);
	if (!config)
		return NULL;

	/* Set header section */
	config->param.apr_hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	config->param.apr_hdr.pkt_size = size;
	config->param.apr_hdr.src_svc = APR_SVC_AFE;
	config->param.apr_hdr.src_domain = APR_DOMAIN_APPS;
	config->param.apr_hdr.src_port = 0;
	config->param.apr_hdr.dest_svc = APR_SVC_AFE;
	config->param.apr_hdr.dest_domain = APR_DOMAIN_ADSP;
	config->param.apr_hdr.dest_port = 0;
	config->param.apr_hdr.token = index;
	config->param.apr_hdr.opcode = AFE_PORT_CMD_SET_PARAM_V2;

	/* Set param section */
	config->param.port_id = (uint16_t) port;
	config->param.mem_hdr.data_payload_addr_lsw = 0;
	config->param.mem_hdr.data_payload_addr_msw = 0;
	config->param.mem_hdr.mem_map_handle = 0;
	/* max data size of the param_ID/module_ID combination */
	config->param.payload_size = payload_size;

	/* Set data section */
	config->data.module_id = (uint32_t) module;
	config->data.param_id = (uint32_t) param;
	config->data.reserved = 0; /* Must be set to 0 */
	/* actual size of the data for the module_ID/param_ID pair */
	config->data.param_size = length;

	return config;
}

static int crus_afe_get_param(int port, int module, int param, int length,
			      void *data)
{
	struct afe_custom_crus_get_config_t *config = NULL;
	int index = afe_get_port_index(port);
	int ret = 0, count = 0;

	pr_debug("port = 0x%x module = 0x%x param = 0x%x length = %d\n",
		port, module, param, length);

	if (!msm_crus_is_cirrus_afe_topology()) {
		pr_warn("afe port is not cirrus's topology");
		return -EPERM;
	}

	config = crus_alloc_afe_get_header(length, port, module, param);
	if (config == NULL) {
		pr_err("Memory allocation failed!\n");
		return -ENOMEM;
	}

	pr_debug("Preparing to send apr packet\n");

	mutex_lock(&crus_sp_get_param_lock);
	atomic_set(&crus_sp_get_param_flag, 0);

	crus_sp_get_buffer_size = config->param.payload_size + 16;
	crus_sp_get_buffer = kzalloc(crus_sp_get_buffer_size, GFP_KERNEL);

	if (!crus_sp_get_buffer) {
		pr_err("kzalloc failed for crus_sp_get_buffer!\n");
		ret = -ENOMEM;
		goto crus_sp_get_buffer_err;
	}

	ret = afe_apr_send_pkt_crus(config, index, 0);
	if (ret)
		pr_err("crus get_param for port 0x%x failed with code %d\n",
			port, ret);
	else
		pr_debug("crus get_param sent packet with param id 0x%08x to module 0x%08x.\n",
			param, module);

	/* Wait for afe callback to populate data */
	while (!atomic_read(&crus_sp_get_param_flag)) {
		usleep_range(1000, 2000);
		if (count++ >= 1000) {
			pr_err("AFE callback timeout\n");
			atomic_set(&crus_sp_get_param_flag, 1);
			ret = -EINVAL;
			goto crus_sp_get_param_err;
		}
	}

	/* Copy from dynamic buffer to return buffer */
	memcpy(data, &crus_sp_get_buffer[4], length);

crus_sp_get_param_err:
	kfree(crus_sp_get_buffer);
	crus_sp_get_buffer = NULL;
	crus_sp_get_buffer_size = -1;

crus_sp_get_buffer_err:
	mutex_unlock(&crus_sp_get_param_lock);
	kfree(config);
	return ret;
}

static int crus_afe_set_param(int port, int module, int param, int length,
			      void *data_ptr)
{
	struct afe_custom_crus_set_config_t *config = NULL;
	int index = afe_get_port_index(port);
	int ret = 0;

	pr_info("port = 0x%x module = 0x%x param = 0x%x length = %d\n",
		port, module, param, length);

	if (!msm_crus_is_cirrus_afe_topology()) {
		pr_warn("afe port is not cirrus's topology");
		return -EPERM;
	}

	config = crus_alloc_afe_set_header(length, port, module, param);
	if (config == NULL) {
		pr_err("Memory allocation failed!\n");
		return -ENOMEM;
	}

	memcpy(config + 1, data_ptr, length);

	pr_debug("Preparing to send apr packet.\n");

	ret = afe_apr_send_pkt_crus(config, index, 1);
	if (ret)
		pr_err("crus set_param for port 0x%x failed with code %d\n",
			port, ret);
	else
		pr_debug("crus set_param sent packet with param id 0x%08x to module 0x%08x.\n",
			param, module);

	kfree(config);
	return ret;
}

static int crus_afe_send_config(const char *data, size_t length,
				int32_t port, int32_t module)
{
	struct afe_custom_crus_set_config_t *config = NULL;
	struct crus_external_config_t *payload = NULL;
	int size = sizeof(struct crus_external_config_t);
	int ret = 0;
	int index = afe_get_port_index(port);
	uint32_t param = 0;
	int sent = 0;
	int chars_to_send = 0;

	pr_info("send config with module_id = %x, string length = %d\n",
		module, length);

	if (!msm_crus_is_cirrus_afe_topology()) {
		pr_warn("afe port is not cirrus's topology");
		return -EPERM;
	}

	/* Destination settings for message */
	if (port == cirrus_ff_port)
		param = CRUS_PARAM_RX_SET_EXT_CONFIG;
	else if (port == cirrus_fb_port)
		param = CRUS_PARAM_TX_SET_EXT_CONFIG;
	else {
		pr_err("Received invalid port parameter %d\n", module);
		return -EINVAL;
	}

	config = crus_alloc_afe_set_header(size, port, module, param);

	if (config == NULL) {
		pr_err("Memory allocation failed!\n");
		return -ENOMEM;
	}

	payload = (struct crus_external_config_t *)(config + 1);
	payload->total_size = (uint32_t)length;
	payload->reserved = 0;
	payload->config = PAYLOAD_FOLLOWS_CONFIG;
	    /*   This tells the algorithm to expect array */
	    /*   immediately following the header */

	/* Send config string in chunks of APR_CHUNK_SIZE bytes */
	while (sent < length) {
		chars_to_send = length - sent;
		if (chars_to_send > APR_CHUNK_SIZE) {
			chars_to_send = APR_CHUNK_SIZE;
			payload->done = 0;
		} else {
			payload->done = 1;
		}

		/* Configure per message parameter settings */
		memcpy(payload->data, data + sent, chars_to_send);
		payload->chunk_size = chars_to_send;

		/* Send the actual message */
		pr_debug("Preparing to send apr packet.\n");
		ret = afe_apr_send_pkt_crus(config, index, 1);

		if (ret)
			pr_err("crus set_param for port %d failed with code %d\n",
				port, ret);
		else
			pr_debug("crus set_param sent packet with param id 0x%08x to module 0x%08x.\n",
				param, module);

		sent += chars_to_send;
	}

	kfree(config);
	return ret;
}

static int crus_afe_send_delta(const char *data, size_t length)
{
	struct afe_custom_crus_set_config_t *config = NULL;
	struct crus_delta_config_t *payload = NULL;
	int size = sizeof(struct crus_delta_config_t);
	int port = cirrus_ff_port;
	int param = CRUS_PARAM_RX_SET_DELTA_CONFIG;
	int module = CIRRUS_SP;
	int ret = 0;
	int index = afe_get_port_index(port);
	int sent = 0;
	int chars_to_send = 0;

	pr_info("called with module_id = %x, string length = %d\n",
		module, length);

	if (!msm_crus_is_cirrus_afe_topology()) {
		pr_warn("afe port is not cirrus's topology");
		return -EPERM;
	}

	config = crus_alloc_afe_set_header(size, port, module, param);
	if (config == NULL) {
		pr_err("Memory allocation failed!\n");
		return -ENOMEM;
	}

	payload = (struct crus_delta_config_t *)(config + 1);
	payload->total_size = length;
	payload->index = 0;
	payload->reserved = 0;
	payload->config = PAYLOAD_FOLLOWS_CONFIG;
	    /* ^ This tells the algorithm to expect array */
	    /*   immediately following the header */

	/* Send config string in chunks of APR_CHUNK_SIZE bytes */
	while (sent < length) {
		chars_to_send = length - sent;
		if (chars_to_send > APR_CHUNK_SIZE) {
			chars_to_send = APR_CHUNK_SIZE;
			payload->done = 0;
		} else {
			payload->done = 1;
		}

		/* Configure per message parameter settings */
		memcpy(payload->data, data + sent, chars_to_send);
		payload->chunk_size = chars_to_send;

		/* Send the actual message */
		pr_debug("Preparing to send apr packet.\n");
		ret = afe_apr_send_pkt_crus(config, index, 1);

		if (ret)
			pr_err("crus set_param for port %d failed with code %d\n",
				port, ret);
		else
			pr_debug("crus set_param sent packet with param id 0x%08x to module 0x%08x.\n",
				param, module);

		sent += chars_to_send;
	}

	kfree(config);
	return ret;
}

extern int crus_afe_callback(void *payload, int size)
{
	uint32_t *payload32 = payload;
	int copysize;

	pr_debug("Cirrus AFE CALLBACK: size = %d\n", size);
	if (size < 8)
		return -EINVAL;

	switch (payload32[1]) {
	case CIRRUS_SP:
		if (crus_sp_get_buffer != NULL) {
			copysize = (crus_sp_get_buffer_size > size)?
				size : crus_sp_get_buffer_size;

			if (copysize != size)
				pr_warn("size mismatch data may lost\n");

			memcpy(crus_sp_get_buffer, payload32, copysize);
			atomic_set(&crus_sp_get_param_flag, 1);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(crus_afe_callback);

static int msm_crus_send_usecase(int usecase)
{
	struct crus_rx_run_case_ctrl_t case_ctrl;

	case_ctrl.status_l = 1;
	case_ctrl.status_r = 1;
	case_ctrl.z_l = crus_sp_cal_rslt.z_l;
	case_ctrl.z_r = crus_sp_cal_rslt.z_r;
	case_ctrl.checksum_l = crus_sp_cal_rslt.z_l + 1;
	case_ctrl.checksum_r = crus_sp_cal_rslt.z_r + 1;

	case_ctrl.atemp = 23;
	case_ctrl.value = usecase;

	if (crus_afe_set_param(cirrus_fb_port, CIRRUS_SP,
			   CRUS_PARAM_TX_SET_USECASE, sizeof(usecase),
			   &usecase))
		return -EPERM;

	if (crus_afe_set_param(cirrus_ff_port, CIRRUS_SP,
			   CRUS_PARAM_RX_SET_USECASE, sizeof(case_ctrl),
			   &case_ctrl))
		return -EPERM;

	return 0;
}

static int msm_crus_send_config(int usecase, int port,
				const struct firmware *firmware)
{
	int ret = 0;

	if (firmware == NULL)
		return -EINVAL;

	if (usecase >= crus_sp_usecase_dt_count)
		return -EINVAL;

	if (port == SPK_RX) {
		if (count_config & (1 << (SPK_RX + usecase * 2)))
			return 0;


		if (msm_crus_send_usecase(usecase)) {
			ret = -EPERM;
			goto exit;
		}

		if (crus_afe_send_config(
				firmware->data,
				firmware->size,
				cirrus_ff_port, CIRRUS_SP)) {
			ret = -EPERM;
			goto exit;
		}

		count_config |= (1 << (SPK_RX + usecase * 2));
	} else if (port == SPK_TX) {
		if (count_config & (1 << (SPK_TX + usecase * 2)))
			return 0;

		if (msm_crus_send_usecase(usecase)) {
			ret = -EPERM;
			goto exit;
		}

		if (crus_afe_send_config(
				firmware->data,
				firmware->size,
				cirrus_fb_port, CIRRUS_SP)) {
			ret = -EPERM;
			goto exit;
		}

		count_config |= (1 << (SPK_TX + usecase * 2));
	} else {
		pr_err("unknown port %d", port);
		ret = -EINVAL;
	}

exit:
	msm_crus_send_usecase(cirrus_sp_usecase);
	return ret;
}

int msm_routing_cirrus_fbport_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cirrus_fb_port_ctl;
	return 0;
}

int msm_routing_cirrus_fbport_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	cirrus_fb_port_ctl = ucontrol->value.integer.value[0];

	switch (cirrus_fb_port_ctl) {
	case 0:
		cirrus_fb_port = AFE_PORT_ID_PRIMARY_MI2S_TX;
		cirrus_ff_port = AFE_PORT_ID_PRIMARY_MI2S_RX;
		break;
	case 1:
		cirrus_fb_port = AFE_PORT_ID_SECONDARY_MI2S_TX;
		cirrus_ff_port = AFE_PORT_ID_SECONDARY_MI2S_RX;
		break;
	case 2:
		cirrus_fb_port = AFE_PORT_ID_TERTIARY_MI2S_TX;
		cirrus_ff_port = AFE_PORT_ID_TERTIARY_MI2S_RX;
		break;
	case 3:
		cirrus_fb_port = AFE_PORT_ID_QUATERNARY_MI2S_TX;
		cirrus_ff_port = AFE_PORT_ID_QUATERNARY_MI2S_RX;
		break;
	case 4:
		cirrus_fb_port = AFE_PORT_ID_TERTIARY_TDM_TX;
		cirrus_ff_port = AFE_PORT_ID_TERTIARY_TDM_RX;
		break;
	default:
		/* Default port to TERTIARY */
		cirrus_fb_port_ctl = 4;
		cirrus_fb_port = AFE_PORT_ID_TERTIARY_TDM_TX;
		cirrus_ff_port = AFE_PORT_ID_TERTIARY_TDM_RX;
		break;
	}
	return 0;
}

static int msm_routing_crus_sp_enable(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	const int crus_set = ucontrol->value.integer.value[0];
	int ret = 0;

	if (crus_set > 255) {
		pr_err("Cirrus SP Enable: Invalid entry; Enter 0 to DISABLE, 1 to ENABLE; 2-255 are reserved for debug\n");
		return -EINVAL;
	}

	mutex_lock(&crus_sp_lock);

	switch (crus_set) {
	case 0: /* "Config SP Disable" */
		pr_info("Cirrus SP Enable: Config DISABLE\n");
		crus_enable.value = 0;
		cirrus_sp_en = 0;
		break;
	case 1: /* "Config SP Enable" */
		pr_info("Cirrus SP Enable: Config ENABLE\n");
		crus_enable.value = 1;
		cirrus_sp_en = 1;
		break;
	default:
		ret = -EINVAL;
		goto exit;
	}

	crus_spk.flag = crus_spk.flag & ~CIRRUS_ENABLE;

	if (crus_afe_set_param(cirrus_ff_port, CIRRUS_SP,
				CIRRUS_SP_ENABLE,
				sizeof(struct crus_single_data_t),
				(void *)&crus_enable)) {
		ret = -EPERM;
		goto exit;
	}

	if (crus_afe_set_param(cirrus_fb_port, CIRRUS_SP,
				CIRRUS_SP_ENABLE,
				sizeof(struct crus_single_data_t),
				(void *)&crus_enable)) {
		ret = -EPERM;
		goto exit;
	}

	crus_spk.flag = crus_spk.flag | CIRRUS_ENABLE;

exit:
	mutex_unlock(&crus_sp_lock);
	pr_debug("flag: %x ret: %d", crus_spk.flag, ret);

	return ret;
}

static int msm_routing_crus_sp_enable_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cirrus_sp_en;
	return 0;
}

static bool msm_crus_check_count(unsigned char number, int total_count)
{
	pr_debug("number: %x, total_count: %d", number, total_count);

	return hweight8(number) == total_count;
}

static int msm_routing_crus_sp_usecase(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	const int crus_set = ucontrol->value.integer.value[0];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	uint32_t max_index = e->items;
	int ret = 0;

	pr_debug("Starting Cirrus SP Config function call %d\n", crus_set);

	if (crus_set >= max_index) {
		pr_err("Cirrus SP Config index out of bounds (%d)\n", crus_set);
		return -EINVAL;
	}

	mutex_lock(&crus_sp_lock);

	cirrus_sp_usecase = crus_set;
	crus_spk.flag = crus_spk.flag & ~CIRRUS_USECASE;

	if (msm_crus_send_usecase(cirrus_sp_usecase)) {
		ret = -EPERM;
		goto exit;
	}

	crus_spk.flag = crus_spk.flag | CIRRUS_USECASE;

exit:
	mutex_unlock(&crus_sp_lock);
	pr_debug("flag: %x ret: %d", crus_spk.flag, ret);

	return ret;
}

static int msm_routing_crus_sp_usecase_get(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cirrus_sp_usecase;

	return 0;
}

static int msm_routing_crus_load_config(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	const int crus_set = ucontrol->value.integer.value[0];
	char *config = NULL;
	const struct firmware *firmware = NULL;
	struct snd_soc_platform *platform = snd_soc_kcontrol_platform(kcontrol);
	struct msm_pcm_drv_data *pdata = snd_soc_platform_get_drvdata(platform);
	int rc = 0;
	int i = 0;

	mutex_lock(&crus_sp_lock);

	config = kzalloc(CONFIG_FILE_SIZE, GFP_KERNEL);
	if (!config) {
		rc = -ENOMEM;
		goto exit;
	}

	pr_debug("Starting Cirrus SP Load Config function call %d\n", crus_set);

	switch (crus_set) {
	case 0:
		break;
	case 1: /* Load RX Config */
		cirrus_fb_load_conf_sel = crus_set;

		for (i = 0; i < crus_sp_usecase_dt_count; i++) {
			if (crus_spk.bin_files[SPK_RX][i]) {
				pr_debug("stored RX firmware usecase: %d already",
					i);
				continue;
			}

			if (i == 0)
				snprintf(config, CONFIG_FILE_SIZE,
						CRUS_RX_CONFIG,
						pdata->config_name);
			else
				snprintf(config, CONFIG_FILE_SIZE,
						CRUS_RX_MULTI_CONFIG,
						pdata->config_name, i);

			if (request_firmware(&firmware, config,
					crus_sp_device) != 0) {
				pr_err("Request firmware %s failed\n", config);
				continue;
			}

			pr_info("Sending RX config\n");
			crus_spk.bin_files[SPK_RX][i] = firmware;

			if (msm_crus_send_config(i, SPK_RX,
					crus_spk.bin_files[SPK_RX][i]))
				continue;
		}
		break;
	case 2: /* Load TX Config */
		cirrus_fb_load_conf_sel = crus_set;

		for (i = 0; i < crus_sp_usecase_dt_count; i++) {
			if (crus_spk.bin_files[SPK_TX][i]) {
				pr_debug("stored TX firmware usecase: %d already",
					i);
				continue;
			}

			if (i == 0)
				snprintf(config, CONFIG_FILE_SIZE,
						CRUS_TX_CONFIG,
						pdata->config_name);
			else
				snprintf(config, CONFIG_FILE_SIZE,
						CRUS_TX_MULTI_CONFIG,
						pdata->config_name, i);

			if (request_firmware(&firmware, config,
					crus_sp_device) != 0) {
				pr_err("Request firmware %s failed\n", config);
				continue;
			}

			pr_info("Sending TX config\n");
			crus_spk.bin_files[SPK_TX][i] = firmware;

			if (msm_crus_send_config(i, SPK_TX,
					crus_spk.bin_files[SPK_TX][i]))
				continue;
		}
		break;
	default:
		rc = -EINVAL;
		goto exit;
	}

	if (msm_crus_check_count(count_config,
			MAX_SPK_PORT * crus_sp_usecase_dt_count)) {
		count_config = 0;
		crus_spk.flag |= CIRRUS_BIN_FILE;
	}
exit:
	cirrus_fb_load_conf_sel = 0;
	kfree(config);
	mutex_unlock(&crus_sp_lock);
	pr_debug("flag: %x rc: %d", crus_spk.flag, rc);
	return rc;
}

static int msm_routing_crus_load_config_get(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cirrus_fb_load_conf_sel;

	return 0;
}

static int msm_routing_crus_delta_config(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct crus_single_data_t data;
	const int crus_set = ucontrol->value.integer.value[0];
	const struct firmware *firmware;

	pr_debug("Starting Cirrus SP Delta Config function call %d\n",
		 crus_set);

	switch (crus_set) {
	case 0:
		break;
	case 1: /* Load delta config over AFE */
		cirrus_fb_delta_sel = crus_set;
		if (request_firmware(&firmware, "crus_sp_delta_config.bin",
				     crus_sp_device) != 0) {
			pr_err("Request firmware failed\n");
			cirrus_fb_delta_sel = 0;
			return -EINVAL;
		}

		pr_info("Sending delta config\n");
		crus_afe_send_delta(firmware->data, firmware->size);
		release_firmware(firmware);
		break;
	case 2: /* Run delta transition */
		cirrus_fb_delta_sel = crus_set;
		data.value = 0;
		crus_afe_set_param(cirrus_ff_port, CIRRUS_SP,
				   CRUS_PARAM_RX_RUN_DELTA_CONFIG,
				   sizeof(struct crus_single_data_t), &data);
		break;
	default:
		return -EINVAL;
	}

	cirrus_fb_delta_sel = 0;
	return 0;
}

static int msm_routing_crus_delta_config_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cirrus_fb_delta_sel;

	return 0;
}

static int msm_routing_crus_chan_swap(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct crus_dual_data_t data;
	const int crus_set = ucontrol->value.integer.value[0];

	pr_debug("Starting Cirrus SP Channel Swap function call %d\n",
		 crus_set);

	switch (crus_set) {
	case 0: /* L/R */
		data.data1 = 1;
		break;
	case 1: /* R/L */
		data.data1 = 2;
		break;
	default:
		return -EINVAL;
	}

	data.data2 = cirrus_ff_chan_swap_dur;

	crus_afe_set_param(cirrus_ff_port, CIRRUS_SP,
			   CRUS_PARAM_RX_CHANNEL_SWAP,
			   sizeof(struct crus_dual_data_t), &data);

	cirrus_ff_chan_swap_sel = crus_set;

	return 0;
}

static int msm_routing_crus_chan_swap_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cirrus_ff_chan_swap_sel;

	return 0;
}

static int msm_routing_crus_chan_swap_dur(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	int crus_set = ucontrol->value.integer.value[0];

	pr_debug("Starting Cirrus SP Channel Swap Duration function call\n");

	if ((crus_set < 0) || (crus_set > MAX_CHAN_SWAP_SAMPLES)) {
		pr_err("Value out of range (%d)\n", crus_set);
		return -EINVAL;
	}

	if (crus_set < MIN_CHAN_SWAP_SAMPLES) {
		pr_info("Received %d, rounding up to min value %d\n",
			crus_set, MIN_CHAN_SWAP_SAMPLES);
		crus_set = MIN_CHAN_SWAP_SAMPLES;
	}

	cirrus_ff_chan_swap_dur = crus_set;

	return 0;
}

static int msm_routing_crus_chan_swap_dur_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cirrus_ff_chan_swap_dur;

	return 0;
}

static int msm_routing_crus_fail_det(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	cirrus_fail_detect_en = ucontrol->value.integer.value[0];
	return 0;
}

static int msm_routing_crus_fail_det_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cirrus_fail_detect_en;
	return 0;
}

static char const *cirrus_fb_port_text[] = {"PRI_MI2S_RX", "SEC_MI2S_RX",
					    "TERT_MI2S_RX", "QUAT_MI2S_RX",
					    "TERT_TDM_RX_0"};

static char const *crus_en_text[] = {"Config SP Disable", "Config SP Enable"};

static char const *crus_conf_load_text[] = {"Idle", "Load RX", "Load TX"};

static char const *crus_delta_text[] = {"Idle", "Load", "Run"};

static char const *crus_chan_swap_text[] = {"LR", "RL"};

static const struct soc_enum cirrus_fb_controls_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, cirrus_fb_port_text),
};

static const struct soc_enum crus_en_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, crus_en_text),
};

static struct soc_enum crus_sp_usecase_enum[] = {
	SOC_ENUM_SINGLE_EXT(MAX_TUNING_CONFIGS, crus_sp_usecase_dt_text),
};

static const struct soc_enum crus_conf_load_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, crus_conf_load_text),
};

static const struct soc_enum crus_delta_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, crus_delta_text),
};

static const struct soc_enum crus_chan_swap_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, crus_chan_swap_text),
};

static const struct snd_kcontrol_new crus_mixer_controls[] = {
	SOC_ENUM_EXT("Cirrus SP FBPort", cirrus_fb_controls_enum[0],
	msm_routing_cirrus_fbport_get, msm_routing_cirrus_fbport_put),
	SOC_ENUM_EXT("Cirrus SP", crus_en_enum[0],
	msm_routing_crus_sp_enable_get, msm_routing_crus_sp_enable),
	SOC_ENUM_EXT("Cirrus SP Usecase", crus_sp_usecase_enum[0],
	msm_routing_crus_sp_usecase_get, msm_routing_crus_sp_usecase),
	SOC_ENUM_EXT("Cirrus SP Load Config", crus_conf_load_enum[0],
	msm_routing_crus_load_config_get, msm_routing_crus_load_config),
	SOC_ENUM_EXT("Cirrus SP Delta Config", crus_delta_enum[0],
	msm_routing_crus_delta_config_get, msm_routing_crus_delta_config),
	SOC_ENUM_EXT("Cirrus SP Channel Swap", crus_chan_swap_enum[0],
	msm_routing_crus_chan_swap_get, msm_routing_crus_chan_swap),
	SOC_SINGLE_EXT("Cirrus SP Channel Swap Duration", SND_SOC_NOPM, 0,
	MAX_CHAN_SWAP_SAMPLES, 0, msm_routing_crus_chan_swap_dur_get,
	msm_routing_crus_chan_swap_dur),
	SOC_SINGLE_BOOL_EXT("Cirrus SP Failure Detection", 0,
	msm_routing_crus_fail_det_get, msm_routing_crus_fail_det),
};

void msm_crus_pb_add_controls(struct snd_soc_platform *platform)
{
	crus_sp_device = platform->dev;

	if (crus_sp_device == NULL)
		pr_err("platform->dev is NULL!\n");
	else
		pr_debug("platform->dev = %lx\n",
			 (unsigned long)crus_sp_device);

	if (crus_sp_usecase_dt_count == 0)
		pr_err("Missing usecase config selections\n");

	crus_sp_usecase_enum[0].items = crus_sp_usecase_dt_count;
	crus_sp_usecase_enum[0].texts = crus_sp_usecase_dt_text;

	snd_soc_add_platform_controls(platform, crus_mixer_controls,
				      ARRAY_SIZE(crus_mixer_controls));
}
EXPORT_SYMBOL(msm_crus_pb_add_controls);

static u32 get_bound_z(u32 z, u32 tolerance, char bound)
{
	if (bound == 'u')
		return z + z * tolerance / 100;
	else
		return z - z * tolerance / 100;
}

static void msm_crus_check_calibration_value(void)
{
	int32_t swap = 0;
	u32 left_lower_bound = 0, left_upper_bound = 0;
	u32 right_lower_bound = 0, right_upper_bound = 0;

	if (swap_calibration) {
		pr_info("swap calibration value");
		swap = crus_sp_cal_rslt.status_l;
		crus_sp_cal_rslt.status_l = crus_sp_cal_rslt.status_r;
		crus_sp_cal_rslt.status_r = swap;

		swap = crus_sp_cal_rslt.checksum_l;
		crus_sp_cal_rslt.checksum_l = crus_sp_cal_rslt.checksum_r;
		crus_sp_cal_rslt.checksum_r = swap;

		swap = crus_sp_cal_rslt.z_l;
		crus_sp_cal_rslt.z_l = crus_sp_cal_rslt.z_r;
		crus_sp_cal_rslt.z_r = swap;
	}

	left_lower_bound = get_bound_z(crus_cal.top_spk_impedance,
			crus_cal.top_spk_tolerance, 'l');
	left_upper_bound = get_bound_z(crus_cal.top_spk_impedance,
			crus_cal.top_spk_tolerance, 'u');

	right_lower_bound = get_bound_z(crus_cal.bottom_spk_impedance,
			crus_cal.bottom_spk_tolerance, 'l');
	right_upper_bound = get_bound_z(crus_cal.bottom_spk_impedance,
			crus_cal.bottom_spk_tolerance, 'u');

	pr_debug("left bound: Lower: %u Upper: %u , right bound Lower: %u Upper: %u",
		left_lower_bound, left_upper_bound,
		right_lower_bound, right_upper_bound);

	if (crus_sp_cal_rslt.z_l > left_upper_bound
		|| crus_sp_cal_rslt.z_l < left_lower_bound) {
		pr_err("left calibartion %u over limit, set default value %u",
			crus_sp_cal_rslt.z_l, crus_cal.top_spk_mean);
		crus_sp_cal_rslt.z_l = crus_cal.top_spk_mean;
	} else
		pr_info("left calibration %u\n", crus_sp_cal_rslt.z_l);

	if (crus_sp_cal_rslt.z_r > right_upper_bound
		|| crus_sp_cal_rslt.z_r < right_lower_bound) {
		pr_err("right calibartion %u over limit, set default value %u",
			crus_sp_cal_rslt.z_r, crus_cal.bottom_spk_mean);
		crus_sp_cal_rslt.z_r = crus_cal.bottom_spk_mean;
	} else
		pr_info("right calibration %u\n", crus_sp_cal_rslt.z_r);
}

int msm_crus_store_imped(char channel)
{
	/* cs35l36 speaker amp constant value */
	const int amp_factor = CRUS_AMP_FACTOR;
	const int scale_factor = 100000000;
	int32_t *buffer = NULL;
	int out_cal0;
	int out_cal1;
	int ret = 0;

	if (!msm_crus_is_cirrus_afe_topology())
		return -EINVAL;

	buffer = kzalloc(CRUS_RX_GET_TEMP_BUFFER_SIZE, GFP_KERNEL);
	if (buffer == NULL)
		return -ENOMEM;

	if (crus_afe_get_param(cirrus_ff_port,
		CIRRUS_SP, CRUS_PARAM_RX_GET_TEMP,
		CRUS_RX_GET_TEMP_BUFFER_SIZE, buffer)) {
		ret = -EINVAL;
		goto err;
	}

	if (channel == 'l') {
		out_cal0 = buffer[LEFT_OFFSET_OUT_CAL0];
		out_cal1 = buffer[LEFT_OFFSET_OUT_CAL1];

		if ((out_cal0 != 2) || (out_cal1 != 2)) {
			ret = -EINVAL;
			goto err;
		}

		crus_spk.imp_l =  buffer[LEFT_OFFSET_R] * amp_factor;

		pr_info("%s: left impedance %d.%d ohms", __func__,
			crus_spk.imp_l / scale_factor,
			crus_spk.imp_l % scale_factor);

	} else if (channel == 'r') {
		out_cal0 = buffer[RIGHT_OFFSET_OUT_CAL0];
		out_cal1 = buffer[RIGHT_OFFSET_OUT_CAL1];

		if ((out_cal0 != 2) || (out_cal1 != 2)) {
			ret = -EINVAL;
			goto err;
		}

		crus_spk.imp_r = buffer[RIGHT_OFFSET_R] * amp_factor;

		pr_info("%s: right impedance %d.%d ohms", __func__,
			crus_spk.imp_r / scale_factor,
			crus_spk.imp_r % scale_factor);

	} else
		pr_err("unknown channel");

err:
	kfree(buffer);
	return ret;
}
EXPORT_SYMBOL(msm_crus_store_imped);

static ssize_t
resistance_left_right_show(struct device *dev,
		struct device_attribute *a, char *buf)
{
	const int scale_factor = 100000000;

	return snprintf(buf, PAGE_SIZE, "%d.%d,%d.%d",
			crus_spk.imp_l / scale_factor,
			crus_spk.imp_l % scale_factor,
			crus_spk.imp_r / scale_factor,
			crus_spk.imp_r % scale_factor);
}

static DEVICE_ATTR_RO(resistance_left_right);

void msm_crus_check_set_setting(unsigned char cmd)
{
	int i = 0, retries = 10;

	pr_debug("%s: cmd: %d", __func__, cmd);
	mutex_lock(&crus_sp_lock);

	if (cmd == CS35L36_UNMUTE) {
		if (crus_spk.flag == CIRRUS_ALL_SETTING) {
			pr_debug("setting is ready");
			goto exit;
		}

		do {
			if (!(crus_spk.flag & CIRRUS_ENABLE)) {
				if (crus_afe_set_param(cirrus_ff_port,
					CIRRUS_SP,
					CIRRUS_SP_ENABLE,
					sizeof(struct crus_single_data_t),
					(void *)&crus_enable))
					continue;

				if (crus_afe_set_param(cirrus_fb_port,
					CIRRUS_SP,
					CIRRUS_SP_ENABLE,
					sizeof(struct crus_single_data_t),
					(void *)&crus_enable))
					continue;

				crus_spk.flag |= CIRRUS_ENABLE;

				pr_debug("send enable: %d successfully",
					crus_enable.value);
			}

			if (!(crus_spk.flag & CIRRUS_USECASE)) {
				if (msm_crus_send_usecase(cirrus_sp_usecase))
					continue;

				crus_spk.flag |= CIRRUS_USECASE;

				pr_debug("send USECASE: %d successfully",
					cirrus_sp_usecase);
			}

			if (!(crus_spk.flag & CIRRUS_BIN_FILE)) {
				for (i = 0; i < crus_sp_usecase_dt_count; i++) {
					if (!crus_spk.bin_files
						[SPK_RX][i] ||
						!crus_spk.bin_files
						[SPK_TX][i]) {
						pr_info("profile: %d is not ready",
							i);
						continue;
					}

					if (msm_crus_send_config(i, SPK_TX,
						crus_spk.bin_files[SPK_TX][i]))
						continue;

					if (msm_crus_send_config(i, SPK_RX,
						crus_spk.bin_files[SPK_RX][i]))
						continue;
				}

				if (msm_crus_check_count(count_config,
						MAX_SPK_PORT *
						crus_sp_usecase_dt_count)) {
					count_config = 0;
					crus_spk.flag |= CIRRUS_BIN_FILE;

					pr_debug("send bin files successfully");
				}
			}
		} while (crus_spk.flag != CIRRUS_ALL_SETTING && retries-- > 0);
	} else if (cmd == AFE_SSR) {
		pr_debug("clear flag");
		crus_spk.flag = 0;
	} else {
		pr_err("%s: unknown command: %d", __func__, cmd);
	}

	if (retries <= 0 && crus_spk.flag != 0 && count_config != 0) {
		pr_err("stop to retry");
		count_config = 0;
		crus_spk.flag = CIRRUS_ALL_SETTING;
	}
exit:
	mutex_unlock(&crus_sp_lock);
	pr_debug("crus_spk.flag: %x", crus_spk.flag);
}
EXPORT_SYMBOL(msm_crus_check_set_setting);

static long crus_sp_shared_ioctl(struct file *f, unsigned int cmd,
				 void __user *arg)
{
	int result = 0, port;
	uint32_t bufsize = 0, size;
	uint32_t option = 0;
	void *io_data = NULL;

	if (copy_from_user(&size, arg, sizeof(size))) {
		pr_err("copy_from_user (size) failed\n");
		result = -EFAULT;
		goto exit;
	}

	if (size != sizeof(crus_sp_hdr)) {
		pr_err("%s: the payload size is invalid", __func__);
		result = -EINVAL;
		goto exit;
	}

	/* Copy IOCTL header from usermode */
	if (copy_from_user(&crus_sp_hdr, arg, size)) {
		pr_err("copy_from_user (struct) failed\n");
		result = -EFAULT;
		goto exit;
	}

	if (crus_sp_hdr.data_length > CRUS_PARAM_TEMP_MAX_LENGTH) {
		pr_err("data_length(%d) invalid\n", crus_sp_hdr.data_length);
		result = -EINVAL;
		goto exit;
	}

	bufsize = crus_sp_hdr.data_length;
	io_data = kzalloc(bufsize, GFP_KERNEL);

	switch (cmd) {
	case CRUS_SP_IOCTL_GET:
		switch (crus_sp_hdr.module_id) {
		case CRUS_MODULE_ID_TX:
			port = cirrus_fb_port;
			break;
		case CRUS_MODULE_ID_RX:
			port = cirrus_ff_port;
			break;
		default:
			pr_info("Unrecognized port ID (%d)\n",
			       crus_sp_hdr.module_id);
			port = cirrus_ff_port;
		}

		crus_afe_get_param(port, CIRRUS_SP, crus_sp_hdr.param_id,
				   bufsize, io_data);

		result = copy_to_user(crus_sp_hdr.data, io_data, bufsize);
		if (result) {
			pr_err("copy_to_user failed (%d)\n", result);
			result = -EFAULT;
		} else {
			result = bufsize;
		}
		break;
	case CRUS_SP_IOCTL_SET:
		result = copy_from_user(io_data, crus_sp_hdr.data, bufsize);
		if (result) {
			pr_err("copy_from_user failed (%d)\n", result);
			result = -EFAULT;
			goto exit_io;
		}

		switch (crus_sp_hdr.module_id) {
		case CRUS_MODULE_ID_TX:
			port = cirrus_fb_port;
			break;
		case CRUS_MODULE_ID_RX:
			port = cirrus_ff_port;
			break;
		default:
			pr_info("Unrecognized port ID (%d)\n",
				crus_sp_hdr.module_id);
			port = cirrus_ff_port;
		}

		crus_afe_set_param(port, CIRRUS_SP, crus_sp_hdr.param_id,
				   bufsize, io_data);
		break;
	case CRUS_SP_IOCTL_GET_CALIB:
		if (copy_from_user(io_data, crus_sp_hdr.data, bufsize)) {
			pr_err("copy_from_user failed\n");
			result = -EFAULT;
			goto exit_io;
		}
		option = 1;
		crus_afe_set_param(cirrus_ff_port, CIRRUS_SP,
				   CRUS_PARAM_RX_SET_CALIB, sizeof(option),
				   (void *)&option);
		crus_afe_set_param(cirrus_fb_port, CIRRUS_SP,
				   CRUS_PARAM_TX_SET_CALIB, sizeof(option),
				   (void *)&option);
		msleep(2000);
		crus_afe_get_param(cirrus_fb_port, CIRRUS_SP,
				   CRUS_PARAM_TX_GET_TEMP_CAL, bufsize,
				   io_data);
		if (copy_to_user(crus_sp_hdr.data, io_data, bufsize)) {
			pr_err("copy_to_user failed\n");
			result = -EFAULT;
		} else {
			result = bufsize;
		}

		break;
	case CRUS_SP_IOCTL_SET_CALIB:
		if (bufsize != sizeof(crus_sp_cal_rslt)) {
			pr_err("%s: the data size is invalid", __func__);
			result = -EINVAL;
			goto exit_io;
		}

		if (copy_from_user(io_data,
				   (void *)crus_sp_hdr.data, bufsize)) {
			pr_err("copy_from_user failed\n");
			result = -EFAULT;
			goto exit_io;
		}

		memcpy(&crus_sp_cal_rslt, io_data, bufsize);

		msm_crus_check_calibration_value();

		break;
	default:
		pr_err("%s: Invalid IOCTL, command = %d!\n", __func__, cmd);
		result = -EINVAL;
	}

exit_io:
	kfree(io_data);
exit:
	return result;
}

static long crus_sp_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
	pr_debug("%s\n", __func__);

	return crus_sp_shared_ioctl(f, cmd, (void __user *)arg);
}

struct compat_crus_sp_ioctl_header {
	uint32_t size;
	uint32_t module_id;
	uint32_t param_id;
	uint32_t data_length;
	compat_caddr_t data;
};

static long crus_sp_compat_ioctl(struct file *f,
				 unsigned int cmd, unsigned long arg)
{
	unsigned int cmd64;
	struct compat_crus_sp_ioctl_header __user *ua32;
	struct crus_sp_ioctl_header __user *ua;
	compat_caddr_t __user ua32_data;
	void __user *ua_data;
	uint32_t ua32_size, ua_size;

	pr_debug("%s\n", __func__);

	ua32 = compat_ptr(arg);
	if (get_user(ua32_size, (uint32_t __user*)&ua32->size))
		return -EFAULT;
	if (ua32_size != sizeof(*ua32))
		return -EINVAL;

	ua_size = sizeof(*ua);
	ua = compat_alloc_user_space(ua_size);
	if (!ua)
		return -ENOMEM;

	/* Copy everything but data, then fixup size & data. */
	if (copy_in_user(ua, ua32, sizeof(*ua32) - sizeof(ua32->data)))
		return -EFAULT;
	if (put_user(ua_size, (uint32_t __user*)&ua->size))
		return -EFAULT;
	if (get_user(ua32_data, (compat_caddr_t __user*)&ua32->data))
		return -EFAULT;
	ua_data = compat_ptr(ua32_data);
	if (put_user(ua_data, (void* __user*)&ua->data))
		return -EFAULT;

	switch (cmd) {
	case CRUS_SP_IOCTL_GET32:
		cmd64 = CRUS_SP_IOCTL_GET;
		break;
	case CRUS_SP_IOCTL_SET32:
		cmd64 = CRUS_SP_IOCTL_SET;
		break;
	case CRUS_SP_IOCTL_GET_CALIB32:
		cmd64 = CRUS_SP_IOCTL_GET_CALIB;
		break;
	case CRUS_SP_IOCTL_SET_CALIB32:
		cmd64 = CRUS_SP_IOCTL_SET_CALIB;
		break;
	default:
		pr_err("%s: Invalid IOCTL, command = %d!\n", __func__, cmd);
		return -EINVAL;
	}

	return crus_sp_shared_ioctl(f, cmd64, ua);
}

static int crus_sp_open(struct inode *inode, struct file *f)
{
	int result = 0;

	pr_debug("%s\n", __func__);

	atomic_inc(&crus_sp_misc_usage_count);
	return result;
}

static int crus_sp_release(struct inode *inode, struct file *f)
{
	int result = 0;

	pr_debug("%s\n", __func__);

	atomic_dec(&crus_sp_misc_usage_count);
	pr_debug("ref count %d!\n", atomic_read(&crus_sp_misc_usage_count));

	return result;
}

static ssize_t
temperature_left_show(struct device *dev, struct device_attribute *a, char *buf)
{
	static const int material = MAGIC_MATERIAL;
	static const int scale_factor = 100000;
	int32_t *buffer = NULL;
	int out_cal0;
	int out_cal1;
	int z, r, t;
	int temp0;

	buffer = kzalloc(CRUS_RX_GET_TEMP_BUFFER_SIZE, GFP_KERNEL);
	if (buffer == NULL) {
		return -ENOMEM;
	}

	crus_afe_get_param(cirrus_ff_port, CIRRUS_SP, CRUS_PARAM_RX_GET_TEMP,
			   CRUS_RX_GET_TEMP_BUFFER_SIZE, buffer);

	out_cal0 = buffer[LEFT_OFFSET_OUT_CAL0];
	out_cal1 = buffer[LEFT_OFFSET_OUT_CAL1];

	z = buffer[LEFT_OFFSET_Z];

	temp0 = buffer[LEFT_OFFSET_TEMP];

	if ((out_cal0 != 2) || (out_cal1 != 2)) {
		kfree(buffer);
		return snprintf(buf, PAGE_SIZE, "Calibration is not done\n");
	}

	r = buffer[LEFT_OFFSET_R];
	t = (material * scale_factor * (r - z) / z) + (temp0 * scale_factor);

	kfree(buffer);
	return snprintf(buf, PAGE_SIZE, "%d.%05dc\n",
			t / scale_factor, t % scale_factor);
}

static DEVICE_ATTR_RO(temperature_left);

static ssize_t
temperature_right_show(struct device *dev, struct device_attribute *a,
		       char *buf)
{
	static const int material = MAGIC_MATERIAL;
	static const int scale_factor = 100000;
	int32_t *buffer = NULL;
	int out_cal0;
	int out_cal1;
	int z, r, t;
	int temp0;

	buffer = kzalloc(CRUS_RX_GET_TEMP_BUFFER_SIZE, GFP_KERNEL);
	if (buffer == NULL) {
		return -ENOMEM;
	}

	crus_afe_get_param(cirrus_ff_port, CIRRUS_SP, CRUS_PARAM_RX_GET_TEMP,
			   CRUS_RX_GET_TEMP_BUFFER_SIZE, buffer);

	out_cal0 = buffer[RIGHT_OFFSET_OUT_CAL0];
	out_cal1 = buffer[RIGHT_OFFSET_OUT_CAL1];

	z = buffer[RIGHT_OFFSET_Z];

	temp0 = buffer[RIGHT_OFFSET_TEMP];

	if ((out_cal0 != 2) || (out_cal1 != 2)) {
		kfree(buffer);
		return snprintf(buf, PAGE_SIZE, "Calibration is not done\n");
	}

	r = buffer[RIGHT_OFFSET_R];
	t = (material * scale_factor * (r - z) / z) + (temp0 * scale_factor);

	kfree(buffer);
	return snprintf(buf, PAGE_SIZE, "%d.%05dc\n",
			t / scale_factor, t % scale_factor);
}

static DEVICE_ATTR_RO(temperature_right);

static ssize_t
resistance_left_show(struct device *dev, struct device_attribute *a, char *buf)
{
	static const int scale_factor = 100000000;
	static const int amp_factor = CRUS_AMP_FACTOR;
	int32_t *buffer = NULL;
	int out_cal0;
	int out_cal1;
	int r;

	buffer = kzalloc(CRUS_RX_GET_TEMP_BUFFER_SIZE, GFP_KERNEL);
	if (buffer == NULL) {
		return -ENOMEM;
	}

	crus_afe_get_param(cirrus_ff_port, CIRRUS_SP, CRUS_PARAM_RX_GET_TEMP,
			   CRUS_RX_GET_TEMP_BUFFER_SIZE, buffer);

	out_cal0 = buffer[LEFT_OFFSET_OUT_CAL0];
	out_cal1 = buffer[LEFT_OFFSET_OUT_CAL1];

	if ((out_cal0 != 2) || (out_cal1 != 2)) {
		kfree(buffer);
		return snprintf(buf, PAGE_SIZE, "Calibration is not done\n");
	}

	r = buffer[LEFT_OFFSET_R] * amp_factor;

	kfree(buffer);
	return snprintf(buf, PAGE_SIZE, "%d.%08d ohms\n",
			r / scale_factor, r % scale_factor);
}

static DEVICE_ATTR_RO(resistance_left);

static ssize_t
resistance_right_show(struct device *dev, struct device_attribute *a, char *buf)
{
	static const int scale_factor = 100000000;
	static const int amp_factor = CRUS_AMP_FACTOR;
	int32_t *buffer = NULL;
	int out_cal0;
	int out_cal1;
	int r;

	buffer = kzalloc(CRUS_RX_GET_TEMP_BUFFER_SIZE, GFP_KERNEL);
	if (buffer == NULL) {
		return -ENOMEM;
	}

	crus_afe_get_param(cirrus_ff_port, CIRRUS_SP, CRUS_PARAM_RX_GET_TEMP,
			   CRUS_RX_GET_TEMP_BUFFER_SIZE, buffer);

	out_cal0 = buffer[RIGHT_OFFSET_OUT_CAL0];
	out_cal1 = buffer[RIGHT_OFFSET_OUT_CAL1];

	if ((out_cal0 != 2) || (out_cal1 != 2)) {
		kfree(buffer);
		return snprintf(buf, PAGE_SIZE, "Calibration is not done\n");
	}

	r = buffer[RIGHT_OFFSET_R] * amp_factor;

	kfree(buffer);
	return snprintf(buf, PAGE_SIZE, "%d.%08d ohms\n",
			r / scale_factor, r % scale_factor);
}

static DEVICE_ATTR_RO(resistance_right);

static struct attribute *crus_sp_attrs[] = {
	&dev_attr_temperature_left.attr,
	&dev_attr_temperature_right.attr,
	&dev_attr_resistance_left.attr,
	&dev_attr_resistance_right.attr,
	&dev_attr_resistance_left_right.attr,
	NULL,
};

static const struct attribute_group crus_sp_group = {
	.attrs = crus_sp_attrs,
};

static const struct attribute_group *crus_sp_groups[] = {
	&crus_sp_group,
	NULL,
};

static void msm_crus_init_spk(void)
{
	int i = 0, j = 0;

	pr_debug("init ssr parameters");

	mutex_lock(&crus_sp_lock);
	crus_spk.flag = 0;
	for (i = 0; i < MAX_SPK_PORT; i++)
		for (j = 0; j < MAX_TUNING_CONFIGS; j++)
			crus_spk.bin_files[i][j] = NULL;
	mutex_unlock(&crus_sp_lock);
}

static int msm_crus_parse_spk(struct platform_device *pdev)
{
	swap_calibration = of_property_read_bool(pdev->dev.of_node,
				"cirrus,swap_calibration");

	if (of_property_read_u32(pdev->dev.of_node,
		"cirrus,top-speaker-impedance", &crus_cal.top_spk_impedance)) {
		dev_err(&pdev->dev, "parse cirrus,top-speaker-impedance failed\n");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "top of speaker impedance: %u\n",
			crus_cal.top_spk_impedance);

	if (of_property_read_u32(pdev->dev.of_node,
		"cirrus,top-speaker-tolerance", &crus_cal.top_spk_tolerance)) {
		dev_err(&pdev->dev, "parse cirrus,top-speaker-tolerance failed\n");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "top of speaker tolerance: %u\n",
			crus_cal.top_spk_tolerance);

	if (of_property_read_u32(pdev->dev.of_node,
		"cirrus,top-speaker-mean", &crus_cal.top_spk_mean)) {
		dev_err(&pdev->dev, "parse cirrus,top-speaker-mean failed\n");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "top of speaker mean: %u\n",
			crus_cal.top_spk_mean);


	if (of_property_read_u32(pdev->dev.of_node,
		"cirrus,bottom-speaker-impedance",
		&crus_cal.bottom_spk_impedance)) {
		dev_err(&pdev->dev, "parse cirrus,bottom-speaker-impedance failed\n");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "bottom of speaker impedance: %u\n",
			crus_cal.bottom_spk_impedance);

	if (of_property_read_u32(pdev->dev.of_node,
		"cirrus,bottom-speaker-tolerance",
		&crus_cal.bottom_spk_tolerance)) {
		dev_err(&pdev->dev, "parse cirrus,top-speaker-tolerance failed\n");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "bottom of speaker tolerance: %u\n",
			crus_cal.bottom_spk_tolerance);

	if (of_property_read_u32(pdev->dev.of_node,
		"cirrus,bottom-speaker-mean", &crus_cal.bottom_spk_mean)) {
		dev_err(&pdev->dev, "parse cirrus,bottom-speaker-mean failed\n");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "bottom of speaker mean: %u\n",
			crus_cal.bottom_spk_mean);

	return 0;
}
static int msm_cirrus_playback_probe(struct platform_device *pdev)
{
	int i;

	pr_info("CRUS_SP: initializing platform device\n");

	crus_sp_usecase_dt_count = of_property_count_strings(pdev->dev.of_node,
							     "usecase-names");
	if (crus_sp_usecase_dt_count <= 0) {
		dev_dbg(&pdev->dev, "Usecase names not found\n");
		crus_sp_usecase_dt_count = 0;
		return 0;
	}

	if ((crus_sp_usecase_dt_count > 0) &&
	    (crus_sp_usecase_dt_count <= MAX_TUNING_CONFIGS))
		of_property_read_string_array(pdev->dev.of_node,
					      "usecase-names",
					      crus_sp_usecase_dt_text,
					      crus_sp_usecase_dt_count);
	else if (crus_sp_usecase_dt_count > MAX_TUNING_CONFIGS) {
		dev_err(&pdev->dev, "Max of %d usecase configs allowed\n",
			MAX_TUNING_CONFIGS);
		return -EINVAL;
	}

	for (i = 0; i < crus_sp_usecase_dt_count; i++)
		pr_info("CRUS_SP: usecase[%d] = %s\n", i,
			 crus_sp_usecase_dt_text[i]);

	if (msm_crus_parse_spk(pdev))
		return -EINVAL;

	msm_crus_init_spk();
	pr_info("CRUS_SP: init ok\n");

	return 0;
}

static const struct of_device_id msm_cirrus_playback_dt_match[] = {
	{.compatible = "cirrus,msm-cirrus-playback"},
	{}
};
MODULE_DEVICE_TABLE(of, msm_cirrus_playback_dt_match);

static struct platform_driver msm_cirrus_playback_driver = {
	.driver = {
		.name = "msm-cirrus-playback",
		.owner = THIS_MODULE,
		.of_match_table = msm_cirrus_playback_dt_match,
	},
	.probe = msm_cirrus_playback_probe,
};

static const struct file_operations crus_sp_fops = {
	.owner = THIS_MODULE,
	.open = crus_sp_open,
	.release = crus_sp_release,
	.unlocked_ioctl = crus_sp_ioctl,
	.compat_ioctl = crus_sp_compat_ioctl,
};

struct miscdevice crus_sp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msm_cirrus_playback",
	.fops = &crus_sp_fops,
};

int __init crus_sp_init(void)
{
	pr_info("CRUS_SP_INIT: initializing misc device\n");
	atomic_set(&crus_sp_get_param_flag, 0);
	atomic_set(&crus_sp_misc_usage_count, 0);
	mutex_init(&crus_sp_get_param_lock);
	mutex_init(&crus_sp_lock);

	misc_register(&crus_sp_misc);

	if (sysfs_create_groups(&crus_sp_misc.this_device->kobj,
				crus_sp_groups))
		pr_err("Could not create sysfs groups\n");

	return platform_driver_register(&msm_cirrus_playback_driver);
}

void __exit crus_sp_exit(void)
{
	int i = 0, j = 0;

	mutex_destroy(&crus_sp_get_param_lock);
	mutex_destroy(&crus_sp_lock);
	for (i = 0; i < MAX_SPK_PORT; i++)
		for (j = 0; j < MAX_TUNING_CONFIGS; j++) {
			release_firmware(crus_spk.bin_files[i][j]);
			crus_spk.bin_files[i][j] = NULL;
		}
	platform_driver_unregister(&msm_cirrus_playback_driver);
}


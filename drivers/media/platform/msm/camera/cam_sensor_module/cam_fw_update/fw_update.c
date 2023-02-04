/* OIS calibration interface for LC898123 F40
 *
 */

#include "fw_update.h"
#include "VCM_firmware.h"

#define OIS_CUR_FW_VERSION           0x08
#define OIS_COMPONENT_I2C_ADDR_WRITE 0x7C
#define OIS_REARWIDE_I2C_ADDR_WRITE  0x7C /*0x76*/
#define OIS_REARTELE_I2C_ADDR_WRITE  0x78
#define VCM_COMPONENT_I2C_ADDR_WRITE 0xE4
#define VCM_EEPROM_I2C_ADDR_WRITE 0xE6
#define MK_SHARP                     0x04170000
#define MK_LGIT                      0x09170000
#define RETRY_MAX 3

static struct camera_io_master *g_io_master_info;
bool g_first = true;
static bool force_disable_LTC;
static bool isRearVCMInitDownload;

/* Implemented in cam_actuator_core.c */
extern bool check_act_ltc_disable(void);

void RamWrite32A(UINT_16 RamAddr, UINT_32 RamData)
{
	int rc = 0;
	struct camera_io_master *io_master_info = g_io_master_info;
	struct cam_sensor_i2c_reg_setting i2c_reg_settings;
	struct cam_sensor_i2c_reg_array i2c_reg_array;

	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	i2c_reg_settings.size = 1;
	i2c_reg_settings.delay = 0;
	i2c_reg_array.reg_addr = RamAddr;
	i2c_reg_array.reg_data = RamData;
	i2c_reg_array.delay = 0;
	i2c_reg_settings.reg_setting = &i2c_reg_array;

	rc = camera_io_dev_write(io_master_info, &i2c_reg_settings);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "[OISFW] %s : write failed\n", __func__);
}

void RamRead32A(UINT_16 RamAddr, UINT_32 *ReadData)
{
	int rc = 0;

	rc = camera_io_dev_read(g_io_master_info, RamAddr,
		ReadData, CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_DWORD);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s read i2c failed\n", __func__);
}

int RamWrite16A(struct camera_io_master *io_info,
	UINT_32 RamAddr, UINT_32 RamData)
{
	int rc = 0;
	struct cam_sensor_i2c_reg_setting i2c_reg_settings;
	struct cam_sensor_i2c_reg_array i2c_reg_array;

	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_settings.size = 1;
	i2c_reg_settings.delay = 0;
	i2c_reg_array.reg_addr = RamAddr;
	i2c_reg_array.reg_data = RamData;
	i2c_reg_array.delay = 0;
	i2c_reg_settings.reg_setting = &i2c_reg_array;

	rc = camera_io_dev_write(io_info, &i2c_reg_settings);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
			"%s: write 0x%x failed", __func__, RamAddr);
	}

	return rc;
}

int RamRead16A(struct camera_io_master *io_info,
	UINT_32 RamAddr, UINT_32 *ReadData)
{
	int rc = 0;

	rc = camera_io_dev_read(io_info, RamAddr,
		ReadData, CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR,
			"%s: read 0x%x failed", __func__, RamAddr);

	return rc;
}

int RamWrite8A(struct camera_io_master *io_info,
	UINT_32 RamAddr, UINT_32 RamData)
{
	int rc = 0;
	struct cam_sensor_i2c_reg_setting i2c_reg_settings;
	struct cam_sensor_i2c_reg_array i2c_reg_array;

	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.size = 1;
	i2c_reg_settings.delay = 0;
	i2c_reg_array.reg_addr = RamAddr;
	i2c_reg_array.reg_data = RamData;
	i2c_reg_array.delay = 0;
	i2c_reg_settings.reg_setting = &i2c_reg_array;

	rc = camera_io_dev_write(io_info, &i2c_reg_settings);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
			"%s: write 0x%x failed", __func__, RamAddr);
	}

	return rc;
}

int RamRead8A(struct camera_io_master *io_info,
	UINT_32 RamAddr, UINT_32 *ReadData)
{
	int rc = 0;

	rc = camera_io_dev_read(io_info, RamAddr,
		ReadData, CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR,
			"%s: read 0x%x failed", __func__, RamAddr);

	return rc;
}

void WitTim(UINT_16 UsWitTim)
{
	msleep(UsWitTim);
}

int CntWrt(UINT_8 *PcSetDat, UINT_16 UsDatNum)
{
	int rc = 0, cnt;
	uint16_t total_bytes = UsDatNum-1;
	uint8_t *ptr = NULL;
	struct camera_io_master *io_master_info = g_io_master_info;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * total_bytes,
		GFP_KERNEL);
	if (!i2c_reg_setting.reg_setting) {
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s Failed in allocating i2c_array",
			__func__);
		return -ENOMEM;
	}

	for (cnt = 0, ptr = &PcSetDat[1]; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =	PcSetDat[0];
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(
		io_master_info, &i2c_reg_setting, 1);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s i2c write sequence error:%d\n",
			__func__, rc);

	kfree(i2c_reg_setting.reg_setting);
	return rc;
}

int CntRd3(UINT_32 addr, void *PcSetDat, UINT_16 UsDatNum)
{
	int rc = 0;

	rc = camera_io_dev_read_seq(g_io_master_info, addr, PcSetDat,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD,
		UsDatNum);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s read i2c failed\n", __func__);
	return rc;
}

void WPBCtrl(UINT_8 UcCtrl)
{
	//do nothing because lc898123F40 uses UnlockCodeSet()
	//to handle WPB by itself
}

int checkHighLevelCommand(int cnt)
{
	int i;
	UINT_32 FWRead;

	for (i = 0; i < cnt ; i++) {
		WitTim(10);
		RamRead32A(0xF100, &FWRead);//Check high level command ready.
		if (FWRead == 0x0) {
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s finish.", __func__);
			return 0;
		}
		CAM_INFO(CAM_SENSOR, "[OISFW]:%s waiting...", __func__);
	}
	CAM_ERR(CAM_SENSOR, "[OISFW]:%s fail.", __func__);
	return -EINVAL;
}

int doFWupdate(UINT_16 CAL_ID, UINT_32 MODULE_MAKER)
{
	int rc = 0;

	if (MODULE_MAKER == MK_SHARP) {
		if (CAL_ID == 0x0) {
			CAM_INFO(CAM_SENSOR,
				"[OISFW]:%s not to update FW because of unknown CAL_ID.",
				__func__);
			rc = -EINVAL;
		} else if (CAL_ID == 0x1) {
			CAM_INFO(CAM_SENSOR,
				"[OISFW]:%s F40_FlashDownload(0, 4, 0 )",
				__func__);
			rc = F40_FlashDownload(0, 4, 0);
		} else {
			CAM_INFO(CAM_SENSOR,
				"[OISFW]:%s F40_FlashDownload(0, 4, 1 )",
				__func__);
			rc = F40_FlashDownload(0, 4, 1);
		}
	} else if (MODULE_MAKER == MK_LGIT) {
		if (CAL_ID == 0x0) {
			CAM_INFO(CAM_SENSOR,
				"[OISFW]:%s not to update FW because of unknown CAL_ID.",
				__func__);
			rc = -EINVAL;
		} else if (CAL_ID == 0x1) {
			CAM_INFO(CAM_SENSOR,
				"[OISFW]:%s F40_FlashDownload(0, 9, 0 )",
				__func__);
			rc = F40_FlashDownload(0, 9, 0);
		} else {
			CAM_INFO(CAM_SENSOR,
				"[OISFW]:%s F40_FlashDownload(0, 9, 1 )",
				__func__);
			rc = F40_FlashDownload(0, 9, 1);
		}
	} else {
		CAM_ERR(CAM_SENSOR,
			"[OISFW]:%s unknown module maker.", __func__);
		rc = -EINVAL;
	}

	if (rc == 0) {
		/*Wait for FW update finish.*/
		rc = checkHighLevelCommand(20);
	} else {
		CAM_ERR(CAM_SENSOR,
			"[OISFW]%s: OIS FW update failed rc = %d.\n",
			__func__, rc);
		rc = -EINVAL;
	}

	return rc;
}

bool checkOISFWversion(UINT_16 *cal_id, UINT_32 *module_maker)
{
	int rc;
	UINT_16 RamAddr;
	UINT_32 UlReadVal;
	UINT_16 FW_version;
	bool need_update = false;

	CAM_INFO(CAM_SENSOR, "[OISFW]:%s\n", __func__);

	RamAddr = 0x8000;
	RamRead32A(RamAddr, &UlReadVal);
	FW_version = UlReadVal & 0xFF;
	*module_maker = UlReadVal & 0xFFFF0000;
	CAM_INFO(CAM_SENSOR, "[OISFW]:%s module_version =  0x%02x.\n",
		__func__, UlReadVal);

	RamAddr = 0x8004;
	RamRead32A(RamAddr, &UlReadVal);
	*cal_id = UlReadVal & 0xFF;
	CAM_INFO(CAM_SENSOR,
		"[OISFW]:%s CAL_ID = 0x%04x, MODULE_MAKER = 0x%x\n",
		__func__, *cal_id, *module_maker);

	if (FW_version >= OIS_CUR_FW_VERSION) {
		CAM_INFO(CAM_SENSOR,
			"[OISFW]%s: No need to update.\n", __func__);
	} else {
		rc = checkHighLevelCommand(100);
		if (rc != 0) {
			CAM_ERR(CAM_SENSOR,
				"[OISFW]:%s checkHighLevelCommand failed = %d\n",
				__func__, rc);
		} else
			need_update = true;
	}

	return need_update;
}

int checkOISFWUpdate(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	int i;
	unsigned short cci_client_sid_backup;
	UINT_32 FWRead;
	UINT_16 FW_version;
	UINT_16 cal_id;
	UINT_32 module_maker;

	if (g_first != true) {
		CAM_INFO(CAM_SENSOR, "[OISFW]%s: No need.\n", __func__);
		return 0;
	}
	g_first = false;

	CAM_INFO(CAM_SENSOR, "[OISFW]:%s 1. sid = %d\n", __func__,
		s_ctrl->io_master_info.cci_client->sid);

	/* Bcakup the I2C slave address */
	cci_client_sid_backup = s_ctrl->io_master_info.cci_client->sid;

	/* Replace the I2C slave address with OIS component */
	s_ctrl->io_master_info.cci_client->sid =
		OIS_COMPONENT_I2C_ADDR_WRITE >> 1;

	g_io_master_info = &(s_ctrl->io_master_info);
	WitTim(100);

	/*Check current HW and FW version*/
	if (checkOISFWversion(&cal_id, &module_maker) == true) {
		rc = doFWupdate(cal_id, module_maker);

		for (i = 0; i < 2 ; i++) {
			RamRead32A(0x8000, &FWRead);
			FW_version = FWRead & 0xFF;
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s 0x8000 = 0x%08x",
				__func__, FWRead);
			if (FW_version != OIS_CUR_FW_VERSION) {
				CAM_ERR(CAM_SENSOR,
					"[OISFW]:FW version check failed after update. retry.\n");
				rc = doFWupdate(cal_id, module_maker);
			} else {
				CAM_INFO(CAM_SENSOR,
					"[OISFW]: FW vserion verify pass.\n");
				break;
			}
		}
	}

	/* Restore the I2C slave address */
	s_ctrl->io_master_info.cci_client->sid =
		cci_client_sid_backup;
	CAM_INFO(CAM_SENSOR, "[OISFW]:%s 2. sid = %d\n", __func__,
		s_ctrl->io_master_info.cci_client->sid);

	CAM_INFO(CAM_SENSOR, "[OISFW]:%s rc = %d\n", __func__, rc);

	return rc;
}

int checkRearVCMFWUpdate_temp(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	UINT_32 retry = 0;
	UINT_32 size = 0;
	UINT_32 regdata = 0;
	unsigned short cci_client_sid_backup;
	uint32_t paramver = -1;
	struct cam_sensor_i2c_reg_array *fwtable = NULL;
	struct device_node *src_node = NULL;
	struct device_node *of_node = s_ctrl->of_node;

	force_disable_LTC = true;
	src_node = of_parse_phandle(of_node, "actuator-src", 0);
	if (!src_node) {
		return 0;
	} else {
		rc = of_property_read_u32(src_node,
			"param-index", &paramver);
		CAM_INFO(CAM_SENSOR, "[VCMFW] paramver=%d, rc=%d",
			paramver, rc);
		of_node_put(src_node);
		src_node = NULL;
	}

	if (paramver == 5) {
		force_disable_LTC = false;
		fwtable = VCM_LC898219_Temp_Params_verE;
		size = sizeof(VCM_LC898219_Temp_Params_verE) /
			sizeof(struct cam_sensor_i2c_reg_array);
	} else if (paramver == 8) {
		force_disable_LTC = false;
		fwtable = VCM_LC898219_Temp_Params_verH;
		size = sizeof(VCM_LC898219_Temp_Params_verH) /
			sizeof(struct cam_sensor_i2c_reg_array);
	} else {
		CAM_INFO(CAM_SENSOR, "[VCMFW] Not to update LTC param");
		return 0;
	}

	cci_client_sid_backup = s_ctrl->io_master_info.cci_client->sid;

	/* Check if need to update EEPROM data */
	rc = ValidateRearVCMFW(&(s_ctrl->io_master_info),
		fwtable, size);
	if (rc == 0) {
		CAM_INFO(CAM_SENSOR,
			"[VCMFW]No need to update firmware");
		goto temp_restore_data;
	}

	/* 1. Replace slave address with 0xE4, do AF firmware update */
	s_ctrl->io_master_info.cci_client->sid =
		VCM_COMPONENT_I2C_ADDR_WRITE >> 1;
	WitTim(8);

	for (retry = 0; retry < RETRY_MAX; retry++) {
		rc = RamRead8A(&(s_ctrl->io_master_info),
			0xF0, &regdata);
		if (regdata == 0xA5) {
			WitTim(1);
			rc = camera_io_dev_poll(
				&(s_ctrl->io_master_info), 0xE0, 0x00, 0,
				CAMERA_SENSOR_I2C_TYPE_BYTE,
				CAMERA_SENSOR_I2C_TYPE_BYTE, 100);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"[VCMFW]i2c poll 0xE0 failed");
				continue;
			}

			/* Update EEPROM data*/
			rc = DownloadRearVCMFW(&(s_ctrl->io_master_info),
				fwtable, size);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"[VCMFW]Download firmware failed");
				continue;
			}

			/* Validate EEPROM data*/
			rc = ValidateRearVCMFW(&(s_ctrl->io_master_info),
				fwtable, size);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"[VCMFW]Validate firmware failed");
				continue;
			}
			CAM_INFO(CAM_SENSOR,
				"[VCMFW]%s: FW update success", __func__);
			break;
		} else {
			CAM_ERR(CAM_SENSOR,
				"[VCMFW]%s: NG module !\n", __func__);
			break;
		}
	}

temp_restore_data:
	/* 2. Restore the I2C slave address */
	s_ctrl->io_master_info.cci_client->sid =
		cci_client_sid_backup;
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: restore sid = %d\n",
		__func__, s_ctrl->io_master_info.cci_client->sid);

	CAM_INFO(CAM_SENSOR, "[VCMFW]%s; rc = %d\n", __func__, rc);

	return rc;
}

int is_force_disable_LTC(void)
{
	return force_disable_LTC;
}

int checkRearVCMLTC(struct camera_io_master *io_info)
{
	int rc = -1;
	UINT_32 retry = 0;
	UINT_32 regdata = 0;
	unsigned short cci_client_sid_backup;
	UINT_32 EEPROM_3Fh = 0;
	UINT_32 UlReadVal;
	UINT_16 init_temp;
	UINT_16 init_temp_A;
	UINT_16 init_temp_B;

	if (io_info->cci_client->sid != (VCM_COMPONENT_I2C_ADDR_WRITE >> 1)) {
		return rc;
	}

	if(is_force_disable_LTC() == true) {
		CAM_INFO(CAM_SENSOR, "[LTC] force disable LTC by project");
		return rc;
	}

	if(check_act_ltc_disable() == true) {
		CAM_INFO(CAM_SENSOR, "[LTC] dynamic disable LTC by user");
		return rc;
	}

	CAM_INFO(CAM_SENSOR, "[LTC] isRearVCMInitDownload=%d",
		isRearVCMInitDownload);
	if (isRearVCMInitDownload == true)
		return rc;

	/* 0. Backup the I2C slave address */
	cci_client_sid_backup = io_info->cci_client->sid;

	/* Read EEPROM 3Fh first */
	io_info->cci_client->sid = VCM_EEPROM_I2C_ADDR_WRITE >> 1;
	RamRead8A(io_info, 0x3F, &EEPROM_3Fh);

	/* 1. Replace slave address with 0xE4 */
	io_info->cci_client->sid = VCM_COMPONENT_I2C_ADDR_WRITE >> 1;

	/* Communication Check */
	RamRead8A(io_info, 0xF0, &regdata);
	if (regdata == 0xA5) {
		CAM_INFO(CAM_SENSOR,
			"[LTC] check communication success");
	} else {
		CAM_ERR(CAM_SENSOR,
			"[LTC] check communication error");
		goto ltc_restore_data;
	}
	usleep_range(1000, 1010);

	/* Enable Temperature Acquisition */
	RamWrite8A(io_info, 0x8E, 0x15);
	RamWrite8A(io_info, 0x8D, 0x20);

	/* Standby Release */
	RamWrite8A(io_info, 0x81, 0x80);
	usleep_range(1000, 1010);

	/* Read Initial Temperature Data */
	RamRead16A(io_info, 0x0058, &UlReadVal);
	init_temp = (UINT_16)UlReadVal;
	CAM_INFO(CAM_SENSOR, "[LTC] init_temp = 0x%x",
		init_temp);

	/* Calculate Shift-Up Gain */
	if ( ((EEPROM_3Fh & 0x38) >> 3) == 2)
		init_temp_A = init_temp << 2;
	else if ( ((EEPROM_3Fh & 0x38) >> 3) == 1)
		init_temp_A = init_temp << 1;
	else
		init_temp_A = init_temp;

	if ( (EEPROM_3Fh & 0x07) == 2)
		init_temp_B = init_temp << 2;
	else if ( (EEPROM_3Fh & 0x07) == 1)
		init_temp_B = init_temp << 1;
	else
		init_temp_B = init_temp;

	/* Initial Data Download */
	RamWrite8A(io_info, 0xE0, 0x01);
	msleep(8); /* wait 8 ms */

	/* LSI wake up check */
	for (retry = 0; retry < 10; retry++) {
		RamRead8A(io_info, 0xB3, &UlReadVal);
		if ( (UlReadVal & 0XE0) == 0 ) {
			break;
		} else {
			if (retry >= 9) {
				CAM_ERR(CAM_SENSOR,
					"[LTC] LSI wake up check failed");
				goto ltc_restore_data;
			}
		}
		usleep_range(1000, 1010);
	}

	/* Write Init Temperature Data */
	RamWrite16A(io_info, 0x30, init_temp_A);
	RamWrite16A(io_info, 0x32, init_temp_A);
	RamWrite16A(io_info, 0x76, init_temp_B);
	RamWrite16A(io_info, 0x78, init_temp_B);

	/* Enable Lens Temp Correction Function */
	RamWrite8A(io_info, 0x8C, 0xE9);
	CAM_INFO(CAM_SENSOR, "[LTC] Enable LTC Function");

	isRearVCMInitDownload = true;
	rc = 0;

ltc_restore_data:
	/* 2. Restore the I2C slave address */
	io_info->cci_client->sid = cci_client_sid_backup;

	return rc;
}

void clearRearVCMInitDownload(struct camera_io_master *io_info)
{
	if (io_info->cci_client->sid == (VCM_COMPONENT_I2C_ADDR_WRITE >> 1)) {
		isRearVCMInitDownload = false;
		CAM_INFO(CAM_SENSOR, "[LTC] clear isRearVCMInitDownload");
	}
}

int DownloadRearVCMFW(struct camera_io_master *io_info,
	struct cam_sensor_i2c_reg_array *fwtable, UINT_32 tbsize)
{
	int rc = 0;
	int i = 0;
	UINT_32 checkbusy_cnt = 0;
	UINT_32 regdata = 0;
	UINT_32 backup_data = 0;

	/* Backup standby setting */
	rc = RamRead8A(io_info, 0x81, &backup_data);
	if (rc < 0) {
		return rc;
	}
	regdata = backup_data & 0x7F;

	rc = RamWrite8A(io_info, 0x81, regdata);
	if (rc < 0) {
		goto restore_data;
	}
	WitTim(100);

	/* Enable EEPROM write mode */
	rc = RamWrite8A(io_info, 0x98, 0xE2);
	if (rc < 0) {
		goto restore_data;
	}
	rc = RamWrite8A(io_info, 0x99, 0xAE);
	if (rc < 0) {
		goto restore_data;
	}
	WitTim(1);

	/* EEPROM data writing */
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: flash firmware, size %d",
		__func__, tbsize);
	for (i = 0; i < tbsize; i++) {
		checkbusy_cnt = 0;
		io_info->cci_client->sid = VCM_EEPROM_I2C_ADDR_WRITE >> 1;
		rc = RamWrite8A(io_info,
			fwtable[i].reg_addr, fwtable[i].reg_data);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"[VCMFW]%s: write data failed, rc = %d",
				__func__, rc);
			goto restore_data;
		}
		io_info->cci_client->sid = VCM_COMPONENT_I2C_ADDR_WRITE >> 1;
		do {
			WitTim(5);
			rc = RamRead8A(io_info, 0xE1, &regdata);
			checkbusy_cnt++;
		} while (((regdata & 0x01) != 0) && checkbusy_cnt < RETRY_MAX);

		if (checkbusy_cnt == RETRY_MAX) {
			CAM_ERR(CAM_SENSOR,
				"[VCMFW]%s: write data warning!", __func__);
		}
	}
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: write data complete.", __func__);

restore_data:
	/* Disable EEPROM write mode & restore standby setting */
	io_info->cci_client->sid = VCM_COMPONENT_I2C_ADDR_WRITE >> 1;
	rc = RamWrite8A(io_info, 0x98, 0x00);
	rc = RamWrite8A(io_info, 0x99, 0x00);
	rc = RamWrite8A(io_info, 0x81, backup_data);
	return rc;
}

int ValidateRearVCMFW(struct camera_io_master *io_info,
	struct cam_sensor_i2c_reg_array *fwtable, UINT_32 tbsize)
{
	int rc = 0;
	int i = 0;

	io_info->cci_client->sid = VCM_EEPROM_I2C_ADDR_WRITE >> 1;
	for (i = 0; i < tbsize; i++) {
		rc = camera_io_dev_poll(io_info,
			fwtable[i].reg_addr, fwtable[i].reg_data, 0,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"[VCMFW]i2c poll 0x%x failed",
				fwtable[i].reg_addr);
			return -EINVAL;
		}
	}
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: complete.", __func__);

	return rc;
}

int GyroReCalib(struct camera_io_master *io_master_info,
	stReCalib *cal_result)
{
	int rc;
	stReCalib pReCalib;

	g_io_master_info = io_master_info;
	if (g_io_master_info == NULL)
		return -EINVAL;

	rc = F40_GyroReCalib(&pReCalib);
	memcpy(cal_result, &pReCalib, sizeof(stReCalib));
	if (rc != 0)
		return rc;

	CAM_INFO(CAM_SENSOR,
		"[OISCali]%d, FctryOffX = %d(0x%x), FctryOffY = %d(0x%x)",
		rc, cal_result->SsFctryOffX, cal_result->SsFctryOffX,
		cal_result->SsFctryOffY, cal_result->SsFctryOffY);
	CAM_INFO(CAM_SENSOR,
		"[OISCali]%d, RecalOffX = %d(0x%x), RecalOffY = %d(0x%x)",
		rc, cal_result->SsRecalOffX, cal_result->SsRecalOffX,
		cal_result->SsRecalOffY, cal_result->SsRecalOffY);
	CAM_INFO(CAM_SENSOR,
		"[OISCali]%d, DiffX = %d(0x%x), DiffY = %d(0x%x)",
		rc, cal_result->SsDiffX, cal_result->SsDiffX,
		cal_result->SsDiffY, cal_result->SsDiffY);

	if (abs(cal_result->SsRecalOffX) >= 0x600 ||
		abs(cal_result->SsRecalOffY) >= 0x600 ||
		abs(cal_result->SsDiffX) >= 0x1000 ||
		abs(cal_result->SsDiffY) >= 0x1000) {
		CAM_ERR(CAM_SENSOR,
			"[OISCali] Check failed.");
		return -EINVAL;
	}
	return rc;
}

int WrGyroOffsetData(void)
{
	int rc;

	rc = F40_WrGyroOffsetData();
	return rc;
}

int getFWVersion(struct cam_sensor_ctrl_t *s_ctrl)
{
	int       rc = 0;
	uint32_t  RamAddr, UlReadVal;
	unsigned short cci_client_sid_backup;

	if (s_ctrl->sensordata->slave_info.sensor_id != 0x363) {
		return -EINVAL;
	}

	/* Bcakup the I2C slave address */
	cci_client_sid_backup = s_ctrl->io_master_info.cci_client->sid;

	/* Replace the I2C slave address with OIS component */
	if (s_ctrl->sensordata->slave_info.sensor_id == 0x363) {
		s_ctrl->io_master_info.cci_client->sid =
			OIS_REARWIDE_I2C_ADDR_WRITE >> 1;
	}

	/* read FW version */
	RamAddr = 0x8000;
	rc = camera_io_dev_read(&s_ctrl->io_master_info, RamAddr,
		&UlReadVal, CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_DWORD);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "[FW] read i2c failed");
	else {
		s_ctrl->ois_fw_ver = UlReadVal & 0xFF;
		s_ctrl->vcm_fw_ver = UlReadVal & 0xFF;
		CAM_INFO(CAM_SENSOR,
			"ois_fwver=0x%02x, vcm_fwver=0x%02x\n",
			s_ctrl->ois_fw_ver, s_ctrl->vcm_fw_ver);
	}

	/* Restore the I2C slave address */
	s_ctrl->io_master_info.cci_client->sid =
		cci_client_sid_backup;

	return rc;
}

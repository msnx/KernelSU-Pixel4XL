/**
 * @brief		LC898123F40 Flash update
 *
 * @author		Copyright (C) 2017, ON Semiconductor, all right reserved.
 *
 **/

//===============================================
//  Include Header File
//===============================================
#include "PhoneUpdate.h"
#include "PmemCode.h"
/* Actuator calibration parameters */
#include "Calibration_Eve.h"   // Wide module
#include "Calibration_Emily.h" // Tele module

#if MODULE_VENDOR == 0x04
#include "FromCode_04_17_02_00.h"
#include "FromCode_04_17_02_01.h"
#elif MODULE_VENDOR == 0x09
#include "FromCode_09_17_02_00.h"
#include "FromCode_09_17_02_01.h"
#else
#include "FromCode_04_17_02_00.h"
#include "FromCode_04_17_02_01.h"
#include "FromCode_09_17_02_00.h"
#include "FromCode_09_17_02_01.h"
#endif

#define USER_RESERVE            3
#define ERASE_BLOCKS            (16 - USER_RESERVE)
#define BURST_LENGTH            (8*5)
#define DMB_COEFF_ADDRESS       0x21
#define BLOCK_UNIT              0x200
#define BLOCK_BYTE              2560
#define SECTOR_SIZE             320
#define HALF_SECTOR_ADD_UNIT    0x20
#define FLASH_ACCESS_SIZE       32
#define USER_AREA_START         (BLOCK_UNIT * ERASE_BLOCKS)

#define CNT100MS                1352
#define CNT200MS                2703

//===============================================
//  CUSTOMER NECESSARY CREATING FUNCTION LIST
//===============================================
/* for I2C communication */
extern void RamWrite32A(UINT_16, UINT_32);
extern void RamRead32A(UINT_16, void *);
/* for I2C Multi Translation : Burst Mode*/
extern void CntWrt(void *, UINT_16);
extern void CntRd3(UINT_32, void *, UINT_16);

/* WPB control for LC898123F40*/
extern void WPBCtrl(UINT_8);
/* for Wait timer [Need to adjust for your system] */
extern void WitTim(UINT_16);

//==========================
//  extern  Function LIST
//==========================
UINT_32 UlBufDat[64];

//==========================
//  Table of download file
//==========================
const DOWNLOAD_TBL DTbl[] = {
#if MODULE_VENDOR == 0x04
	{0x0402, CcMagicCodeF40_04_17_02_00, sizeof(CcMagicCodeF40_04_17_02_00),
		CcFromCodeF40_04_17_02_00, sizeof(CcFromCodeF40_04_17_02_00)},
	{0x0482, CcMagicCodeF40_04_17_02_01, sizeof(CcMagicCodeF40_04_17_02_01),
		CcFromCodeF40_04_17_02_01, sizeof(CcFromCodeF40_04_17_02_01)},
#elif MODULE_VENDOR == 0x09
	{0x0902, CcMagicCodeF40_09_17_02_00, sizeof(CcMagicCodeF40_09_17_02_00),
		CcFromCodeF40_09_17_02_00, sizeof(CcFromCodeF40_09_17_02_00)},
	{0x0982, CcMagicCodeF40_09_17_02_01, sizeof(CcMagicCodeF40_09_17_02_01),
		CcFromCodeF40_09_17_02_01, sizeof(CcFromCodeF40_09_17_02_01)},
#else
	{0x0402, CcMagicCodeF40_04_17_02_00, sizeof(CcMagicCodeF40_04_17_02_00),
		CcFromCodeF40_04_17_02_00, sizeof(CcFromCodeF40_04_17_02_00)},
	{0x0482, CcMagicCodeF40_04_17_02_01, sizeof(CcMagicCodeF40_04_17_02_01),
		CcFromCodeF40_04_17_02_01, sizeof(CcFromCodeF40_04_17_02_01)},
	{0x0902, CcMagicCodeF40_09_17_02_00, sizeof(CcMagicCodeF40_09_17_02_00),
		CcFromCodeF40_09_17_02_00, sizeof(CcFromCodeF40_09_17_02_00)},
	{0x0982, CcMagicCodeF40_09_17_02_01, sizeof(CcMagicCodeF40_09_17_02_01),
		CcFromCodeF40_09_17_02_01, sizeof(CcFromCodeF40_09_17_02_01)},
#endif
	{0xFFFF, (void *)0, 0, (void *)0, 0}
};

//==========================
//  Local Function Prototype
//==========================
void F40_IORead32A(UINT_32 IOadrs, UINT_32 *IOdata)
{
	RamWrite32A(F40_IO_ADR_ACCESS, IOadrs);
	RamRead32A(F40_IO_DAT_ACCESS, IOdata);
}

void F40_IOWrite32A(UINT_32 IOadrs, UINT_32 IOdata)
{
	RamWrite32A(F40_IO_ADR_ACCESS, IOadrs);
	RamWrite32A(F40_IO_DAT_ACCESS, IOdata);
}

UINT_8 F40_ReadWPB(void)
{
	UINT_32 UlReadVal, UlCnt = 0;

	do {
		F40_IORead32A(FLASHROM_F40_WPB, &UlReadVal);
		if ((UlReadVal & 0x00000004) != 0)
			return 1;
		WitTim(1);
	} while (UlCnt++ < 10);
	return 0;
}

UINT_8 F40_UnlockCodeSet(void)
{
	UINT_32 UlReadVal;

	WPBCtrl(WPB_OFF);
	if (F40_ReadWPB() != 1)
		return 5;

	F40_IOWrite32A(FLASHROM_F40_UNLK_CODE1, 0xAAAAAAAA);
	F40_IOWrite32A(FLASHROM_F40_UNLK_CODE2, 0x55555555);
	F40_IOWrite32A(FLASHROM_F40_RSTB_FLA, 0x00000001);
	F40_IOWrite32A(FLASHROM_F40_CLK_FLAON, 0x00000010);
	F40_IOWrite32A(FLASHROM_F40_UNLK_CODE3, 0x0000ACD5);
	F40_IOWrite32A(FLASHROM_F40_WPB, 0x00000001);
	RamRead32A(F40_IO_DAT_ACCESS, &UlReadVal);

	if ((UlReadVal & 0x00000007) != 7)
		return 1;

	return 0;
}

UINT_8 F40_UnlockCodeClear(void)
{
	UINT_32 UlReadVal;

	F40_IOWrite32A(FLASHROM_F40_WPB, 0x00000010);
	RamRead32A(F40_IO_DAT_ACCESS, &UlReadVal);

	if ((UlReadVal & 0x00000080) != 0)
		return 3;

	WPBCtrl(WPB_ON);

	return 0;
}

void F40_BootMode(void)
{
	F40_IOWrite32A(SYSDSP_REMAP, 0x00001440);
}

void F40_SetTempCompParameters(DOWNLOAD_TBL *ptr, UINT_8 *pNVR1)
{
	UINT_8 uiModule = (ptr->Index & 0x80);
	ADJ_TEMP_COMPENSATION *TempCompPtr;
	UINT_8 *NcDatVal;
	UINT_16 Idx, PIdx;

	PIdx = (ptr->Index & 0x0F) - 1;

	if (uiModule == 0x00) { // Wide
		TempCompPtr =
			(ADJ_TEMP_COMPENSATION *)&Eve_TempCompParameter[PIdx];
	} else if (uiModule == 0x80) { // Tele
		TempCompPtr =
			(ADJ_TEMP_COMPENSATION *)&Emily_TempCompParameter[PIdx];
	}

	Idx = 0x28 * 5 + 1; // rcodeX
#ifdef _BIG_ENDIAN_
	// for BIG ENDIAN SYSTEM
	NcDatVal = (UINT_8 *)&TempCompPtr->rcodeX; // rcodeX
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // rcodeY
	NcDatVal = (UINT_8 *)&TempCompPtr->rcodeY;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // rcodeZ
	NcDatVal = (UINT_8 *)&TempCompPtr->rcodeZ;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shag
	NcDatVal = (UINT_8 *)&TempCompPtr->shag;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shbg
	NcDatVal = (UINT_8 *)&TempCompPtr->shbg;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shcg
	NcDatVal = (UINT_8 *)&TempCompPtr->shcg;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shoutag
	NcDatVal = (UINT_8 *)&TempCompPtr->shoutag;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shoutbg
	NcDatVal = (UINT_8 *)&TempCompPtr->shoutbg;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shab
	NcDatVal = (UINT_8 *)&TempCompPtr->shab;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shac
	NcDatVal = (UINT_8 *)&TempCompPtr->shac;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shaa
	NcDatVal = (UINT_8 *)&TempCompPtr->shaa;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shbb
	NcDatVal = (UINT_8 *)&TempCompPtr->shbb;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shbc
	NcDatVal = (UINT_8 *)&TempCompPtr->shbc;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shba
	NcDatVal = (UINT_8 *)&TempCompPtr->shba;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shcb
	NcDatVal = (UINT_8 *)&TempCompPtr->shcb;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shcc
	NcDatVal = (UINT_8 *)&TempCompPtr->shcc;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // shca
	NcDatVal = (UINT_8 *)&TempCompPtr->shca;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // tab
	NcDatVal = (UINT_8 *)&TempCompPtr->tab;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // tac
	NcDatVal = (UINT_8 *)&TempCompPtr->tac;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // taa
	NcDatVal = (UINT_8 *)&TempCompPtr->taa;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // tbb
	NcDatVal = (UINT_8 *)&TempCompPtr->tbb;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // tbc
	NcDatVal = (UINT_8 *)&TempCompPtr->tbc;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // tba
	NcDatVal = (UINT_8 *)&TempCompPtr->tba;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	Idx++; // TEMPOFF
	NcDatVal = (UINT_8 *)&TempCompPtr->TEMPOFF;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
	pNVR1[Idx++] = *NcDatVal++;	pNVR1[Idx++] = *NcDatVal++;
#else
	// for LITTLE ENDIAN SYSTEM
	NcDatVal = (UINT_8 *)&TempCompPtr->rcodeX;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // rcodeY
	NcDatVal = (UINT_8 *)&TempCompPtr->rcodeY;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // rcodeZ
	NcDatVal = (UINT_8 *)&TempCompPtr->rcodeZ;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shag
	NcDatVal = (UINT_8 *)&TempCompPtr->shag;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shbg
	NcDatVal = (UINT_8 *)&TempCompPtr->shbg;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shcg
	NcDatVal = (UINT_8 *)&TempCompPtr->shcg;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shoutag
	NcDatVal = (UINT_8 *)&TempCompPtr->shoutag;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shoutbg
	NcDatVal = (UINT_8 *)&TempCompPtr->shoutbg;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shab
	NcDatVal = (UINT_8 *)&TempCompPtr->shab;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shac
	NcDatVal = (UINT_8 *)&TempCompPtr->shac;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shaa
	NcDatVal = (UINT_8 *)&TempCompPtr->shaa;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shbb
	NcDatVal = (UINT_8 *)&TempCompPtr->shbb;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shbc
	NcDatVal = (UINT_8 *)&TempCompPtr->shbc;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shba
	NcDatVal = (UINT_8 *)&TempCompPtr->shba;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shcb
	NcDatVal = (UINT_8 *)&TempCompPtr->shcb;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shcc
	NcDatVal = (UINT_8 *)&TempCompPtr->shcc;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // shca
	NcDatVal = (UINT_8 *)&TempCompPtr->shca;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // tab
	NcDatVal = (UINT_8 *)&TempCompPtr->tab;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // tac
	NcDatVal = (UINT_8 *)&TempCompPtr->tac;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // taa
	NcDatVal = (UINT_8 *)&TempCompPtr->taa;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // tbb
	NcDatVal = (UINT_8 *)&TempCompPtr->tbb;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // tbc
	NcDatVal = (UINT_8 *)&TempCompPtr->tbc;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // tba
	NcDatVal = (UINT_8 *)&TempCompPtr->tba;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
	Idx += 5; // TEMPOFF
	NcDatVal = (UINT_8 *)&TempCompPtr->TEMPOFF;
	pNVR1[Idx+3] = *NcDatVal++;	pNVR1[Idx+2] = *NcDatVal++;
	pNVR1[Idx+1] = *NcDatVal++;	pNVR1[Idx+0] = *NcDatVal++;
#endif
}

UINT_8 F40_FlashBlockErase(UINT_32 SetAddress)
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8	ans	= 0;

	if (SetAddress & 0x00010000)
		return 9;

	ans	= F40_UnlockCodeSet();
	if (ans != 0)
		return ans;

	F40_IOWrite32A(FLASHROM_F40_ADR, (SetAddress & 0xFFFFFE00));
	F40_IOWrite32A(FLASHROM_F40_CMD, 0x00000006);

	WitTim(5);

	UlCnt = 0;
	do {
		if (UlCnt++ > 100) {
			ans = 2;
			break;
		}

		F40_IORead32A(FLASHROM_F40_INT, &UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

	F40_UnlockCodeClear();

	return ans;
}

UINT_8 F40_FlashBurstWrite(const UINT_8 *NcDataVal,
	UINT_32 NcDataLength, UINT_32 ScNvrMan)
{
	UINT_32	i, j, UlCnt;
	UINT_8	data[163];
	UINT_32	UlReadVal;
	UINT_8	UcOddEvn;
	UINT_8	Remainder;

	data[0] = 0xF0;
	data[1] = 0x08;
	data[2] = BURST_LENGTH;

	for (i = 0; i < (NcDataLength / BURST_LENGTH); i++) {
		UlCnt = 3;

		UcOddEvn = i % 2;
		data[1] = 0x08 + UcOddEvn;

		for (j = 0; j < BURST_LENGTH; j++)
			data[UlCnt++] = *NcDataVal++;

		CntWrt(data, BURST_LENGTH + 3);
		RamWrite32A(0xF00A,
			(UINT_32) ((BURST_LENGTH / 5) * i + ScNvrMan));
		RamWrite32A(0xF00B, (UINT_32) (BURST_LENGTH / 5));

		RamWrite32A(0xF00C, 4 + 4 * UcOddEvn);
	}

	Remainder = NcDataLength % BURST_LENGTH;
	if (Remainder != 0) {
		data[2] = Remainder;
		UlCnt = 3;
		UcOddEvn = i % 2;
		data[1] = 0x08 + UcOddEvn;

		for (j = 0; j < Remainder; j++)
			data[UlCnt++] = *NcDataVal++;

		CntWrt(data, BURST_LENGTH + 3);
		RamWrite32A(0xF00A,
			(UINT_32) ((BURST_LENGTH / 5) * i + ScNvrMan));
		RamWrite32A(0xF00B, (UINT_32) (Remainder / 5));
		RamWrite32A(0xF00C, 4 + 4 * UcOddEvn);
	}

	UlCnt = 0;
	do {
		if (UlCnt++ > 100)
			return 1;

		RamRead32A(0xF00C, &UlReadVal);
	} while (UlReadVal != 0);

	return 0;
}

void F40_FlashSectorRead(UINT_32 UlAddress, UINT_8 *PucData)
{
	UINT_8 UcIndex, UcNum;
	UINT_8 UcReadDat[4];

	F40_IOWrite32A(FLASHROM_F40_ADR, (UlAddress & 0xFFFFFFC0));
	F40_IOWrite32A(FLASHROM_F40_ACSCNT, 63);
	UcNum = 64;

	F40_IOWrite32A(FLASHROM_F40_CMD, 0x00000001);

	for (UcIndex = 0; UcIndex < UcNum; UcIndex++) {
		RamWrite32A(F40_IO_ADR_ACCESS, FLASHROM_F40_RDATH);
		RamRead32A(F40_IO_DAT_ACCESS, UcReadDat);
		*PucData++ = UcReadDat[0];
		RamWrite32A(F40_IO_ADR_ACCESS, FLASHROM_F40_RDATL);
		RamRead32A(F40_IO_DAT_ACCESS, UcReadDat);
		*PucData++ = UcReadDat[3];
		*PucData++ = UcReadDat[2];
		*PucData++ = UcReadDat[1];
		*PucData++ = UcReadDat[0];
	}
}

UINT_8 F40_FlashInt32Read(UINT_32 UlAddress, UINT_32 *PuiData)
{
	UINT_8 UcResult = 0;

	F40_IOWrite32A(FLASHROM_F40_ADR, UlAddress);
	F40_IOWrite32A(FLASHROM_F40_ACSCNT, 0x00000000);
	F40_IOWrite32A(FLASHROM_F40_CMD, 0x00000001);
	F40_IORead32A(FLASHROM_F40_RDATL, PuiData);

	return UcResult;
}

void F40_CalcChecksum(const UINT_8 *pData, UINT_32 len,
	UINT_32 *pSumH, UINT_32 *pSumL)
{
	UINT_64 sum = 0;
	UINT_32 dat;
	UINT_16 i;

	for (i = 0; i < len / 5; i++) {
		sum += (UINT_64)*pData++ << 32;

		dat = (UINT_32)*pData++ << 24;
		dat += (UINT_32)*pData++ << 16;
		dat += (UINT_32)*pData++ << 8;
		dat += (UINT_32)*pData++;
		sum += (UINT_64)dat;
	}

	*pSumH = (UINT_32)(sum >> 32);
	*pSumL = (UINT_32)(sum & 0xFFFFFFFF);
}

void F40_CalcBlockChksum(UINT_8 num, UINT_32 *pSumH, UINT_32 *pSumL)
{
	UINT_8  SectorData[SECTOR_SIZE];
	UINT_32 top;
	UINT_16 sec;
	UINT_64 sum = 0;
	UINT_32 datH, datL;

	top = num * BLOCK_UNIT;

	for (sec = 0; sec < (BLOCK_BYTE / SECTOR_SIZE); sec++) {
		F40_FlashSectorRead(top + sec * 64, SectorData);

		F40_CalcChecksum(SectorData, SECTOR_SIZE, &datH, &datL);
		sum += ((UINT_64)datH << 32) + datL;
	}

	*pSumH = (UINT_32)(sum >> 32);
	*pSumL = (UINT_32)(sum & 0xFFFFFFFF);
}

UINT_8 F40_FlashDownload(UINT_8 chiperase, UINT_8 ModuleVendor, UINT_8 MdlVer)
{
	DOWNLOAD_TBL *ptr;

	ptr = (DOWNLOAD_TBL *)DTbl;
	while (ptr->Index != 0xFFFF) {
		if (ptr->Index == (((UINT_16)ModuleVendor << 8) + MdlVer))
			break;
		ptr++;
	}
	if (ptr->Index == 0xFFFF)
		return 0xF0;

	return F40_FlashUpdate(chiperase, ptr);
}

UINT_8 F40_FlashUpdate(UINT_8 flag, DOWNLOAD_TBL *ptr)
{
	INT_32  SiWrkVl0, SiWrkVl1;
	INT_32  SiAdrVal;
	const   UINT_8 *NcDatVal;
	UINT_32 UlReadVal, UlCnt;
	UINT_8  ans, i;
	UINT_16 UsChkBlocks;
	UINT_8  UserMagicCode[SECTOR_SIZE];

//----------------------------------------------------
// 0.
//----------------------------------------------------
	WitTim(50);
	F40_IORead32A(SYSDSP_SOFTRES, (UINT_32 *)&SiWrkVl0);
	SiWrkVl0 &= 0xFFFFEFFF;
	F40_IOWrite32A(SYSDSP_SOFTRES, SiWrkVl0);
	RamWrite32A(0xF006, 0x00000000);
	F40_IOWrite32A(SYSDSP_DSPDIV, 0x00000001);
	RamWrite32A(0x0344, 0x00000014);
	SiAdrVal = 0x00100000;
	for (UlCnt = 0; UlCnt < 25; UlCnt++) {
		RamWrite32A(0x0340, SiAdrVal);
		SiAdrVal += 0x00000008;
		RamWrite32A(0x0348, UlPmemCodeF40[UlCnt*5]);
		RamWrite32A(0x034C, UlPmemCodeF40[UlCnt*5+1]);
		RamWrite32A(0x0350, UlPmemCodeF40[UlCnt*5+2]);
		RamWrite32A(0x0354, UlPmemCodeF40[UlCnt*5+3]);
		RamWrite32A(0x0358, UlPmemCodeF40[UlCnt*5+4]);
		RamWrite32A(0x033c, 0x00000001);
	}
	for (UlCnt = 0; UlCnt < 9; UlCnt++)
		CntWrt((INT_8 *)&UpData_CommandFromTable[UlCnt*6], 0x00000006);

//----------------------------------------------------
// 1.
//----------------------------------------------------
	if (flag) {
		ans = F40_UnlockCodeSet();
		if (ans != 0)
			return ans;

		F40_IOWrite32A(FLASHROM_F40_ADR, 0x00000000);
		F40_IOWrite32A(FLASHROM_F40_CMD, 0x00000005);
		WitTim(13);
		UlCnt = 0;
		do {
			if (UlCnt++ > 100) {
				ans = 0x10;
				break;
			}
			F40_IORead32A(FLASHROM_F40_INT, &UlReadVal);
		} while ((UlReadVal & 0x00000080) != 0);

	} else {
		for (i = 0; i < ERASE_BLOCKS; i++) {
			ans	= F40_FlashBlockErase(i * BLOCK_UNIT);
			if (ans != 0)
				return ans;
		}
		ans = F40_UnlockCodeSet();
		if (ans != 0)
			return ans;
	}
//----------------------------------------------------
// 2.
//----------------------------------------------------
	F40_IOWrite32A(FLASHROM_F40_ADR, 0x00010000);
	F40_IOWrite32A(FLASHROM_F40_CMD, 0x00000004);
	WitTim(5);
	UlCnt = 0;
	do {
		if (UlCnt++ > 100) {
			ans = 0x10;
			break;
		}
		F40_IORead32A(FLASHROM_F40_INT, &UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

//----------------------------------------------------
// 3.
//----------------------------------------------------
	F40_FlashBurstWrite(ptr->FromCode, ptr->SizeFromCode, 0);

	ans |= F40_UnlockCodeClear();
	if (ans != 0)
		return ans;

//----------------------------------------------------
// 4.
//----------------------------------------------------
	UsChkBlocks = (ptr->SizeFromCode / 160) + 1;
	RamWrite32A(0xF00A, 0x00000000);
	RamWrite32A(0xF00B, UsChkBlocks);
	RamWrite32A(0xF00C, 0x00000100);

	NcDatVal = ptr->FromCode;
	SiWrkVl0 = 0;
	for (UlCnt = 0; UlCnt < ptr->SizeFromCode; UlCnt++)
		SiWrkVl0 += *NcDatVal++;
	UsChkBlocks *= 160;
	for (; UlCnt < UsChkBlocks; UlCnt++)
		SiWrkVl0 += 0xFF;

	UlCnt = 0;
	do {
		if (UlCnt++ > 100)
			return 6;

		RamRead32A(0xF00C, &UlReadVal);
	} while (UlReadVal != 0);

	RamRead32A(0xF00D, &SiWrkVl1);

	if ((int)SiWrkVl0 != (int)SiWrkVl1)
		return 0x20;

//----------------------------------------------------
// X.
//----------------------------------------------------
	if (!flag) {
		UINT_32 sumH, sumL;
		UINT_16 Idx;

		// if you can use memcpy(), modify code.
		for (UlCnt = 0; UlCnt < ptr->SizeMagicCode; UlCnt++)
			UserMagicCode[UlCnt] = ptr->MagicCode[UlCnt];

		for (UlCnt = 0; UlCnt < USER_RESERVE; UlCnt++) {
			F40_CalcBlockChksum(ERASE_BLOCKS + UlCnt, &sumH, &sumL);
			Idx = (ERASE_BLOCKS + UlCnt) * 2 * 5 + 1 + 40;
			NcDatVal = (UINT_8 *)&sumH;

#ifdef _BIG_ENDIAN_
			// for BIG ENDIAN SYSTEM
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			Idx++;
			NcDatVal = (UINT_8 *)&sumL;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
#else
			// for LITTLE ENDIAN SYSTEM
			UserMagicCode[Idx+3] = *NcDatVal++;
			UserMagicCode[Idx+2] = *NcDatVal++;
			UserMagicCode[Idx+1] = *NcDatVal++;
			UserMagicCode[Idx+0] = *NcDatVal++;
			Idx += 5;
			NcDatVal = (UINT_8 *)&sumL;
			UserMagicCode[Idx+3] = *NcDatVal++;
			UserMagicCode[Idx+2] = *NcDatVal++;
			UserMagicCode[Idx+1] = *NcDatVal++;
			UserMagicCode[Idx+0] = *NcDatVal++;
#endif
		}
		NcDatVal = UserMagicCode;
	} else {
		for (UlCnt = 0; UlCnt < ptr->SizeMagicCode; UlCnt++)
			UserMagicCode[UlCnt] = ptr->MagicCode[UlCnt];
		NcDatVal = UserMagicCode;
	}

//----------------------------------------------------
// X-1.
//----------------------------------------------------
	F40_SetTempCompParameters(ptr, (UINT_8 *)NcDatVal);
//----------------------------------------------------
// 5.
//----------------------------------------------------
	ans = F40_UnlockCodeSet();
	if (ans != 0)
		return ans;
	F40_FlashBurstWrite(NcDatVal, SECTOR_SIZE, 0x00010000);
	F40_UnlockCodeClear();

//----------------------------------------------------
// 6.
//----------------------------------------------------
	RamWrite32A(0xF00A, 0x00010000);
	RamWrite32A(0xF00B, 0x00000002);
	RamWrite32A(0xF00C, 0x00000100);

	SiWrkVl0 = 0;
	for (UlCnt = 0; UlCnt < SECTOR_SIZE; UlCnt++)
		SiWrkVl0 += *NcDatVal++;

	UlCnt = 0;
	do {
		if (UlCnt++ > 100)
			return 6;

		RamRead32A(0xF00C, &UlReadVal);
	} while (UlReadVal != 0);
	RamRead32A(0xF00D, &SiWrkVl1);

	if ((int)SiWrkVl0 != (int)SiWrkVl1)
		return 0x30;

	F40_IOWrite32A(SYSDSP_REMAP, 0x00001000);
	return 0;
}

void F40_ReadCalData(UINT_32 *BufDat, UINT_32 *ChkSum)
{
	UINT_16 UsSize = 0, UsNum;

	*ChkSum = 0;

	while (UsSize < 64) {
		F40_IOWrite32A(FLASHROM_F40_ACSCNT, (FLASH_ACCESS_SIZE - 1));
		F40_IOWrite32A(FLASHROM_F40_ADR, 0x00010040 + UsSize);
		F40_IOWrite32A(FLASHROM_F40_CMD, 1);

		RamWrite32A(F40_IO_ADR_ACCESS, FLASHROM_F40_RDATL);

		for (UsNum = 0; UsNum < FLASH_ACCESS_SIZE; UsNum++) {
			RamRead32A(F40_IO_DAT_ACCESS, &(BufDat[UsSize]));
			*ChkSum += BufDat[UsSize++];
		}
	}
}

UINT_8 F40_GyroReCalib(stReCalib *pReCalib)
{
	UINT_8  UcSndDat;
	UINT_32 UlRcvDat;
	UINT_32 UlGofX, UlGofY;
	UINT_32 UiChkSum;
	UINT_32 UlStCnt = 0;

	F40_ReadCalData(UlBufDat, &UiChkSum);

	RamWrite32A(0xF014, 0x00000000);

	do {
		UcSndDat = F40_RdStatus(1);
	} while (UcSndDat != 0 && (UlStCnt++ < CNT100MS));

	RamRead32A(0xF014, &UlRcvDat);
	UcSndDat = (unsigned char)(UlRcvDat >> 24);

	if (UlBufDat[49] == 0xFFFFFFFF)
		pReCalib->SsFctryOffX = (UlBufDat[19] >> 16);
	else
		pReCalib->SsFctryOffX = (UlBufDat[49] >> 16);

	if (UlBufDat[50] == 0xFFFFFFFF)
		pReCalib->SsFctryOffY = (UlBufDat[20] >> 16);
	else
		pReCalib->SsFctryOffY = (UlBufDat[50] >> 16);

	RamRead32A(0x0278, &UlGofX);
	RamRead32A(0x027C, &UlGofY);

	pReCalib->SsRecalOffX = (UlGofX >> 16);
	pReCalib->SsRecalOffY = (UlGofY >> 16);
	pReCalib->SsDiffX =
		pReCalib->SsFctryOffX - pReCalib->SsRecalOffX;
	pReCalib->SsDiffY =
		pReCalib->SsFctryOffY - pReCalib->SsRecalOffY;

	return UcSndDat;
}

UINT_8 F40_EraseCalData(void)
{
	UINT_32 UlReadVal, UlCnt;
	UINT_8  ans;

	ans = F40_UnlockCodeSet();
	if (ans != 0)
		return ans;

	F40_IOWrite32A(FLASHROM_F40_ADR, 0x00010040);
	F40_IOWrite32A(FLASHROM_F40_CMD, 4);

	WitTim(5);
	UlCnt = 0;
	do {
		if (UlCnt++ > 100) {
			ans = 2;
			break;
		}
		F40_IORead32A(FLASHROM_F40_INT, &UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

	ans = F40_UnlockCodeClear();

	return ans;
}

UINT_8 F40_WrGyroOffsetData(void)
{
	UINT_32 UlFctryX, UlFctryY;
	UINT_32 UlCurrX, UlCurrY;
	UINT_32 UlGofX, UlGofY;
	UINT_32 UiChkSum1, UiChkSum2;
	UINT_32 UlSrvStat, UlOisStat, UlAfTgtCod;
	UINT_8  ans;
	UINT_32 UlStCnt = 0;
	UINT_8  UcSndDat;

	RamRead32A(0xF010, &UlSrvStat);
	RamRead32A(0xF012, &UlOisStat);
	RamRead32A(0xF01A, &UlAfTgtCod);
	RamWrite32A(0xF010, 0x00070000);

	F40_ReadCalData(UlBufDat, &UiChkSum2);

	ans = F40_EraseCalData();
	if (ans == 0) {
		RamRead32A(0x0278, &UlGofX);
		RamRead32A(0x027C, &UlGofY);

		UlCurrX = UlBufDat[19];
		UlCurrY = UlBufDat[20];
		UlFctryX = UlBufDat[49];
		UlFctryY = UlBufDat[50];

		if (UlFctryX == 0xFFFFFFFF)
			UlBufDat[49] = UlCurrX;

		if (UlFctryY == 0xFFFFFFFF)
			UlBufDat[50] = UlCurrY;

		UlBufDat[19] = UlGofX;
		UlBufDat[20] = UlGofY;

		F40_WriteCalData(UlBufDat, &UiChkSum1);

		F40_ReadCalData(UlBufDat, &UiChkSum2);
		if (UiChkSum1 != UiChkSum2)
			ans = 0x10;
	}

	RamWrite32A(0xF010, UlSrvStat);

	do {
		UcSndDat = F40_RdStatus(1);
	} while (UcSndDat != 0 && (UlStCnt++ < CNT200MS));

	if (UlOisStat != 0) {
		RamWrite32A(0xF012, 0x00000001);
		UlStCnt = 0;
		do {
			UcSndDat = F40_RdStatus(1);
		} while (UcSndDat != 0 && (UlStCnt++ < CNT100MS));
	}

	if (UlSrvStat & 0x00000004)
		RamWrite32A(0xF01A, UlAfTgtCod); // Resume AF target

	return ans;
}

UINT_8 F40_RdStatus(UINT_8 UcStBitChk)
{
	UINT_32 UlReadVal;

	RamRead32A(0xF100, &UlReadVal);
	if (UcStBitChk)
		UlReadVal &= 0x01000000;

	if (!UlReadVal)
		return 0x00;
	else
		return 0x01;
}

UINT_8 F40_WriteCalData(UINT_32 *BufDat, UINT_32 *ChkSum)
{
	UINT_16 UsSize = 0, UsNum;
	UINT_8  ans;
	UINT_32 UlReadVal;

	*ChkSum = 0;

	ans = F40_UnlockCodeSet();
	if (ans != 0)
		return ans;

	F40_IOWrite32A(FLASHROM_F40_WDATH, 0x000000FF);

	while (UsSize < 64) {
		F40_IOWrite32A(FLASHROM_F40_ACSCNT, (FLASH_ACCESS_SIZE - 1));
		F40_IOWrite32A(FLASHROM_F40_ADR, 0x00010040 + UsSize);
		F40_IOWrite32A(FLASHROM_F40_CMD, 2);
		for (UsNum = 0; UsNum < FLASH_ACCESS_SIZE; UsNum++) {
			F40_IOWrite32A(FLASHROM_F40_WDATL, BufDat[UsSize]);
			do {
				F40_IORead32A(FLASHROM_F40_INT, &UlReadVal);
			} while ((UlReadVal & 0x00000020) != 0);
			*ChkSum += BufDat[UsSize++];
		}
	}

	ans = F40_UnlockCodeClear();

	return ans;
}

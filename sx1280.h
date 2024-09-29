/**
 * \file sx1280.h
 * \brief Sx1280 Driver Header.
 * \author Brahim Ben Sedrine
 * \version 0.1
 * \date 02 March 2024
 *
 * Header file for sx1280 driver.
 *
 */

#ifndef SX1280_H
#define SX1280_H

#include "main.h"
#include "error.h"


#define D_ucSx1280_iWriteRegisterOpCode		0x18

#define D_ucSx1280_iReadRegisterOpCode		0x19


#define D_ucSx1280_iWriteBufferOpCode		0x1A

#define D_ucSx1280_iReadBufferOpCode		0x1B


#define D_ucSx1280_iSetPacketTypeOpCode		0x8A

#define D_ucSx1280_iGetPacketTypeOpCode		0x03

#define D_ucSx1280_iSetStandbyOpCode		0x80

#define D_ucSx1280_iSetTxOpCode		0x83

#define D_ucSx1280_iSetRxOpCode		0x82



#define D_ucSx1280_iSetRfFrequencyOpCode	0x86

#define D_ucSx1280_iSetTxPowerOpCode		0x8E

#define D_ucSx1280_iSetBufferBaseAddressOpCode	0x8F

#define D_ucSx1280_iSetModulationParamsOpCode	0x8B

#define D_ucSx1280_iSetPacketParamsOpCode	0x8C

#define D_ucSx1280_iSetDioIrqParamsOpCode	0x8D

#define D_ucSx1280_iClearIrqStatusOpCode	0x97

#define D_ucSx1280_iGetIrqStatusOpCode	0x15

#define D_ucSx1280_iGetRxBufferStatusOpCode	0x17

#define D_ucSx1280_iGetRssiInstOpCode	0x1F

#define D_ucSx1280_iGetPacketStatusOpCode	0x1D


#define D_uiSx1280_iCrystalFrequencyInKhz	52000
#define D_uiSx1280_iPllDivider				262144



/**
 *\enum T_eSx1280_ePacketType
 *\brief Enumeration containg the different packet types supported by sx1280.
 */
typedef enum
{
	C_eSx1280_ieGfskMode = 0,      		/* GFSK Mode */
	C_eSx1280_ieLoraMode,          		/* LoRa Mode */
	C_eSx1280_ieRangingEngineMode,      /* Ranging Engine Mode */
	C_eSx1280_ieFlrcMode,          		/* FLRC Mode */
	C_eSx1280_ieBleMode,          		/* BLE Mode */
}T_eSx1280_ePacketType;


/**
 *\enum T_eSx1280_ePacketType
 *\brief Enumeration containg the different packet types supported by sx1280.
 */
typedef enum
{
	C_eSx1280_iCrcOk = 0,      		/* Correct CRC */
	C_eSx1280_iBadCrc,          	/* CRC Error */
}T_eSx1280_eCrcStatus;


/**
 *\enum T_eSx1280_ePacketType
 *\brief Enumeration containg the different packet types supported by sx1280.
 */
typedef struct
{
	int8_t m_cMinRssiInst;
	int8_t m_cMaxRssiInst;
	int8_t m_cSnr;
	T_eSx1280_eCrcStatus m_eCrcStatus;
}T_stSx1280_eRxPacketStatus;


/**
 *\enum T_eSx1280_eStandbyConfig
 *\brief Standby mode configuration.
 */
typedef enum
{
	C_eSx1280_iStandbyRc = 0,      		/* Standby RC */
	C_eSx1280_iStandbyXosc,          	/* Standby XOSC */
}T_eSx1280_eStandbyConfig;


/**
 *\enum T_eSx1280_eLoraSpreadingFactor
 *\brief Enumeration containg the different LoRa Spreading Factors supported by sx1280.
 */
typedef enum
{
	C_eSx1280_iLoraSF5 = 0x50,      		/* SF 5 */
	C_eSx1280_iLoraSF6 = 0x60,      		/* SF 6 */
	C_eSx1280_iLoraSF7 = 0x70,      		/* SF 7 */
	C_eSx1280_iLoraSF8 = 0x80,      		/* SF 8 */
	C_eSx1280_iLoraSF9 = 0x90,      		/* SF 9 */
	C_eSx1280_iLoraSF10 = 0xA0,      		/* SF 10 */
	C_eSx1280_iLoraSF11 = 0xB0,      		/* SF 11 */
	C_eSx1280_iLoraSF12 = 0xC0,      		/* SF 12 */
}T_eSx1280_eLoraSpreadingFactor;



T_eError_eErrorType eSx1280_eWriteRegister_Cmd (uint16_t in_usAddress, uint8_t in_ucData);
T_eError_eErrorType eSx1280_eReadRegister_Cmd (uint16_t in_usAddress, uint8_t *out_pucData);


T_eError_eErrorType eSx1280_ePacketType_Set (T_eSx1280_ePacketType in_ePacketType);
T_eError_eErrorType eSx1280_ePacketType_Get (T_eSx1280_ePacketType *out_pePacketType);

T_eError_eErrorType eSx1280_eStandbyMode_Set (T_eSx1280_eStandbyConfig in_eStandbyConfig);

T_eError_eErrorType eSx1280_eRfFrequencyInKhz_Set (uint32_t in_uiRfFrequencyInKhz);

T_eError_eErrorType eSx1280_eTxPower_Set (int8_t in_cTxPower);

T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void);

T_eError_eErrorType eSx1280_eSendData_Cmd (uint8_t *in_pucBuffer, uint8_t in_ucLength,
										   uint32_t in_uiRfFrequencyInKhz, T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor,
										   int8_t in_cTxPower, uint32_t in_uiTxTimeout);
T_eError_eErrorType eSx1280_eReceiveData_Cmd (uint8_t *out_pucBuffer, uint8_t *out_pucLength,
											  uint32_t in_uiRfFrequencyInKhz, T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor,
											  uint32_t in_uiRxTimeout, T_stSx1280_eRxPacketStatus* out_pstRxPacketStatus);


/* Free 2.4 GHz Ism bands (Free from WiFi and Bluetooth) */
/* Low band: from 2400 to 2401 MHz */
/* High band: from 2481 to 2483.5 MHz */
/* Low channels: 2400105, 2400315, 2400525 and 2400735 KHz */
/* High channels: 2481105, 2481315, 2481525, 2481735, 2481945, 2482155, 2482365, 2482575, 2482785, 2482995 and 2483205 KHz */


#endif // SX1280_H

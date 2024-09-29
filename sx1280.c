/**
 * \file sx1280.c
 * \brief Sx1280 Driver Source.
 * \author Brahim Ben Sedrine
 * \version 0.1
 * \date 07 March 2024
 *
 * Source file for sx1280 driver.
 *
 */

#include "cmsis_os.h"

#include "sx1280.h"
#include "cmd.h"
#include "spi.h"
#include "main.h"


#if(IS_NUCLEO == 1)
	extern SPI_HandleTypeDef hspi3;
#else
	extern SPI_HandleTypeDef hspi2;
#endif

#define D_uiSx1280_iCommandTimeout	1000 // in ms

#define D_uiSx1280_iTxDoneIrqMask				0x0001
#define D_uiSx1280_iRxDoneIrqMask				0x0002
#define D_uiSx1280_iPreambleDetectedIrqMask		0x8000
#define D_uiSx1280_iCrcErrorIrqMask				0x0040

static uint8_t gs_pucSx1280_iSpiTxBuffer[255];
static uint8_t gs_pucSx1280_iSpiRxBuffer[255];


/**
 * \fn T_eError_eErrorType eSx1280_eWriteRegister_Cmd (uint16_t in_usAddress, uint8_t in_ucData)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eSendCommand_Cmd (uint8_t *in_pucTxBuffer, uint8_t *out_pucRxBuffer, uint8_t in_ucLength) {

	uint32_t l_uiWaintingTime;
	T_eError_eErrorType l_eReturnValue;

//	/* Wait for Busy Pin */
//	l_uiWaintingTime = 0;
//	while(HAL_GPIO_ReadPin(D_uiGpio_eE28Busy_GPIO_Port, D_uiGpio_eE28Busy_Pin) == 1) {
//		osDelay(2);
//		l_uiWaintingTime++;
//		if(l_uiWaintingTime > (D_uiSx1280_iCommandTimeout/2)) { /* Wait exceeds D_uiSx1280_iCommandTimeout  */
//			D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d: RF Busy before sending OpCode 0x%x", __LINE__, in_pucTxBuffer[0]);
//			return Failure_RfBusy;
//		}
//	}

	/* Send Command */
	HAL_GPIO_WritePin(D_uiGpio_eE28Nss_GPIO_Port, D_uiGpio_eE28Nss_Pin, 0);
	osDelay(2);
#if(IS_NUCLEO == 1)
	l_eReturnValue = eSpi_eMasterTransmitReceive_Cmd(&hspi3, in_pucTxBuffer, out_pucRxBuffer, in_ucLength);
#else
	l_eReturnValue = eSpi_eMasterTransmitReceive_Cmd(&hspi2, in_pucTxBuffer, out_pucRxBuffer, in_ucLength);
#endif
	osDelay(2);
	HAL_GPIO_WritePin(D_uiGpio_eE28Nss_GPIO_Port, D_uiGpio_eE28Nss_Pin, 1);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d: SPI Fail", __LINE__);
		osDelay(1000);
		HAL_NVIC_SystemReset();
		return l_eReturnValue;
	}

	/* Wait for Busy Pin */
	l_uiWaintingTime = 0;
	while(HAL_GPIO_ReadPin(D_uiGpio_eE28Busy_GPIO_Port, D_uiGpio_eE28Busy_Pin) == 1) {
		osDelay(2);
		l_uiWaintingTime++;
		if(l_uiWaintingTime > (D_uiSx1280_iCommandTimeout/2)) { /* Wait exceeds D_uiSx1280_iCommandTimeout  */
			D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d: RF Busy after sending OpCode 0x%x", __LINE__, in_pucTxBuffer[0]);
			osDelay(1000);
			HAL_NVIC_SystemReset();
			return Failure_RfBusy;
		}
	}

	return Success;
}


/**
 * \fn T_eError_eErrorType eSx1280_eWriteRegister_Cmd (uint16_t in_usAddress, uint8_t in_ucData)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eWriteRegister_Cmd (uint16_t in_usAddress, uint8_t in_ucData) {

	T_eError_eErrorType l_eReturnValue;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iWriteRegisterOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)(in_usAddress >> 8);
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)in_usAddress;
	gs_pucSx1280_iSpiTxBuffer[3] = in_ucData;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 4);

	return l_eReturnValue;
}

/**
 * \fn T_eError_eErrorType eSx1280_eWriteRegister_Cmd (uint16_t in_usAddress, uint8_t in_ucData)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eReadRegister_Cmd (uint16_t in_usAddress, uint8_t *out_pucData) {

	T_eError_eErrorType l_eReturnValue;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iReadRegisterOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)(in_usAddress >> 8);
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)in_usAddress;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 5);

	*out_pucData = gs_pucSx1280_iSpiRxBuffer[4];

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eWriteBuffer_Cmd (uint8_t *in_pucBuffer, uint8_t in_ucLength)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eWriteBuffer_Cmd (uint8_t *in_pucBuffer, uint8_t in_ucLength) {

	T_eError_eErrorType l_eReturnValue;
	uint8_t l_ucOffset = 0x00;
	uint32_t l_uiCnt;

	if(in_ucLength > 253) {
		in_ucLength = 253;
	}

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iWriteBufferOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = l_ucOffset;
	for(l_uiCnt=0; l_uiCnt<in_ucLength; l_uiCnt++) {
		gs_pucSx1280_iSpiTxBuffer[2+l_uiCnt] = in_pucBuffer[l_uiCnt];
	}

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 2+in_ucLength);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eWriteBuffer_Cmd (uint8_t *in_pucBuffer, uint8_t in_ucLength)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eReadBuffer_Cmd (uint8_t *out_pucBuffer, uint8_t in_ucLength) {

	T_eError_eErrorType l_eReturnValue;
	uint8_t l_ucOffset = 0x00;
	uint32_t l_uiCnt;

	if(in_ucLength > 253) {
		in_ucLength = 253;
	}

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iReadBufferOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = l_ucOffset;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 3+in_ucLength);

	for(l_uiCnt=0; l_uiCnt<in_ucLength; l_uiCnt++) {
		out_pucBuffer[l_uiCnt] = gs_pucSx1280_iSpiRxBuffer[3+l_uiCnt];
	}

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_ePacketType_Set (T_eSx1280_ePacketType in_ePacketType)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_ePacketType_Set (T_eSx1280_ePacketType in_ePacketType) {

	T_eError_eErrorType l_eReturnValue;
	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetPacketTypeOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)in_ePacketType;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 2);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_ePacketType_Get (T_eSx1280_ePacketType *out_pePacketType)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_ePacketType_Get (T_eSx1280_ePacketType *out_pePacketType) {

	T_eError_eErrorType l_eReturnValue;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iGetPacketTypeOpCode;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 3);

	*out_pePacketType = gs_pucSx1280_iSpiRxBuffer[2];

	return l_eReturnValue;
}



/**
 * \fn T_eError_eErrorType eSx1280_eStandbyMode_Set (T_eSx1280_eStandbyConfig in_eStandbyConfig)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eStandbyMode_Set (T_eSx1280_eStandbyConfig in_eStandbyConfig) {

	T_eError_eErrorType l_eReturnValue;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetStandbyOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)in_eStandbyConfig;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 2);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eRfFrequencyInKhz_Set (uint32_t in_uiRfFrequencyInKhz)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eRfFrequencyInKhz_Set (uint32_t in_uiRfFrequencyInKhz) {

	T_eError_eErrorType l_eReturnValue;
	uint64_t l_ullRfFrequency;

	l_ullRfFrequency = ((uint64_t)in_uiRfFrequencyInKhz * (uint64_t)D_uiSx1280_iPllDivider) / (uint64_t)D_uiSx1280_iCrystalFrequencyInKhz;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetRfFrequencyOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)(l_ullRfFrequency >> 16);
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)(l_ullRfFrequency >> 8);
	gs_pucSx1280_iSpiTxBuffer[3] = (uint8_t)l_ullRfFrequency;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 4);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eTxPower_Set (int8_t in_cTxPower)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eTxPower_Set (int8_t in_cTxPower) {

	T_eError_eErrorType l_eReturnValue;
	uint8_t l_ucRampTime = 0xE0;
	uint8_t l_ucPower;

	if(in_cTxPower < (-18)) {
		in_cTxPower = -18;
	}
	else if(in_cTxPower > 13) {
		in_cTxPower = 13;
	}
	else {
		/* empty else */
	}

	l_ucPower = in_cTxPower + 18;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetTxPowerOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = l_ucPower;
	gs_pucSx1280_iSpiTxBuffer[2] = l_ucRampTime;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 3);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void) {

	T_eError_eErrorType l_eReturnValue;
	uint8_t l_ucTxBaseAddress = 0x00;
	uint8_t l_ucRxBaseAddress = 0x00;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetBufferBaseAddressOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = l_ucTxBaseAddress;
	gs_pucSx1280_iSpiTxBuffer[2] = l_ucRxBaseAddress;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 3);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eLoraModulationParams_Set (T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor) {

	T_eError_eErrorType l_eReturnValue;
	uint8_t l_ucBandwidth = 0x34; /* 200 kHz */
	uint8_t l_ucCodeRate = 0x04;//0x07; /* LORA_CR_LI_4_7 */

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetModulationParamsOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = in_eLoraSpreadingFactor;
	gs_pucSx1280_iSpiTxBuffer[2] = l_ucBandwidth;
	gs_pucSx1280_iSpiTxBuffer[3] = l_ucCodeRate;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 4);
	if(l_eReturnValue != Success) {
		return l_eReturnValue;
	}

	/* Configure register 0x925 according to the used Spreading Factor */
	switch(in_eLoraSpreadingFactor) {
	case C_eSx1280_iLoraSF5:
	case C_eSx1280_iLoraSF6:
		l_eReturnValue = eSx1280_eWriteRegister_Cmd (0x925, 0x1E);
		break;
	case C_eSx1280_iLoraSF7:
	case C_eSx1280_iLoraSF8:
		l_eReturnValue = eSx1280_eWriteRegister_Cmd (0x925, 0x37);
		break;
	case C_eSx1280_iLoraSF9:
	case C_eSx1280_iLoraSF10:
	case C_eSx1280_iLoraSF11:
	case C_eSx1280_iLoraSF12:
		l_eReturnValue = eSx1280_eWriteRegister_Cmd (0x925, 0x32);
		break;
	default:
		l_eReturnValue = Failure_BadArgument;
		break;
	}
	if(l_eReturnValue != Success) {
		return l_eReturnValue;
	}

	/* Configure register 0x93C: write 0x1 in this register */
	l_eReturnValue = eSx1280_eWriteRegister_Cmd (0x93C, 0x01);

	return l_eReturnValue;
}



/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_ePacketParams_Set (uint8_t in_ucPayloadLength) {

	T_eError_eErrorType l_eReturnValue;

	if(in_ucPayloadLength > 253) {
		in_ucPayloadLength = 253;
	}

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetPacketParamsOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = 0x23; /* Preamble Length: 12 symbols: mant=3, exp =2 = 3*2^2 */
	gs_pucSx1280_iSpiTxBuffer[2] = 0x00; /* Header Type: 0: Explicit Header */
	gs_pucSx1280_iSpiTxBuffer[3] = in_ucPayloadLength; /* Payload Length Length */
	gs_pucSx1280_iSpiTxBuffer[4] = 0x20; /* CRC: 0x20: CRC Enabled */
	gs_pucSx1280_iSpiTxBuffer[5] = 0x40; /* IQ Invert: IQ as defined: not swapped */
	gs_pucSx1280_iSpiTxBuffer[6] = 0; /* Not Used */
	gs_pucSx1280_iSpiTxBuffer[7] = 0; /* Not Used */

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 8);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eTxDioIrqParams_Set (void) {

	T_eError_eErrorType l_eReturnValue;

	uint16_t l_usIrqMask = D_uiSx1280_iTxDoneIrqMask; /* Tx Done */
	uint16_t l_usDio1Mask = D_uiSx1280_iTxDoneIrqMask; /* Map Tx Done on DIO 1 */
	uint16_t l_usDio2Mask = 0x0000; /* No mapping on DIO 2 */
	uint16_t l_usDio3Mask = 0x0000; /* No mapping on DIO 3 */

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetDioIrqParamsOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)(l_usIrqMask >> 8);
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)l_usIrqMask;
	gs_pucSx1280_iSpiTxBuffer[3] = (uint8_t)(l_usDio1Mask >> 8);
	gs_pucSx1280_iSpiTxBuffer[4] = (uint8_t)l_usDio1Mask;
	gs_pucSx1280_iSpiTxBuffer[5] = (uint8_t)(l_usDio2Mask >> 8);
	gs_pucSx1280_iSpiTxBuffer[6] = (uint8_t)l_usDio2Mask;
	gs_pucSx1280_iSpiTxBuffer[7] = (uint8_t)(l_usDio3Mask >> 8);
	gs_pucSx1280_iSpiTxBuffer[8] = (uint8_t)l_usDio3Mask;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 9);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eRxDioIrqParams_Set (void) {

	T_eError_eErrorType l_eReturnValue;

	uint16_t l_usIrqMask = D_uiSx1280_iRxDoneIrqMask | D_uiSx1280_iPreambleDetectedIrqMask | D_uiSx1280_iCrcErrorIrqMask;
	uint16_t l_usDio1Mask = D_uiSx1280_iRxDoneIrqMask | D_uiSx1280_iCrcErrorIrqMask;
	uint16_t l_usDio2Mask = D_uiSx1280_iPreambleDetectedIrqMask; /* No mapping on DIO 2 */
	uint16_t l_usDio3Mask = 0x0000; /* No mapping on DIO 3 */

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetDioIrqParamsOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)(l_usIrqMask >> 8);
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)l_usIrqMask;
	gs_pucSx1280_iSpiTxBuffer[3] = (uint8_t)(l_usDio1Mask >> 8);
	gs_pucSx1280_iSpiTxBuffer[4] = (uint8_t)l_usDio1Mask;
	gs_pucSx1280_iSpiTxBuffer[5] = (uint8_t)(l_usDio2Mask >> 8);
	gs_pucSx1280_iSpiTxBuffer[6] = (uint8_t)l_usDio2Mask;
	gs_pucSx1280_iSpiTxBuffer[7] = (uint8_t)(l_usDio3Mask >> 8);
	gs_pucSx1280_iSpiTxBuffer[8] = (uint8_t)l_usDio3Mask;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 9);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eTx_Set (void) {

	T_eError_eErrorType l_eReturnValue;
	uint8_t l_ucPeriodBase = 0x03; /* 4 ms period base */
	uint16_t l_usPeriodBaseCount = 250; /* Set Tx Timeout to 600 ms: 150*4 */

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetTxOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = l_ucPeriodBase;
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)(l_usPeriodBaseCount >> 8);
	gs_pucSx1280_iSpiTxBuffer[3] = (uint8_t)l_usPeriodBaseCount;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 4);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eRx_Set (void) {

	T_eError_eErrorType l_eReturnValue;
	uint8_t l_ucPeriodBase = 0x00; /* no-timeout */
	uint16_t l_usPeriodBaseCount = 0; /* no timeout */

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iSetRxOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = l_ucPeriodBase;
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)(l_usPeriodBaseCount >> 8);
	gs_pucSx1280_iSpiTxBuffer[3] = (uint8_t)l_usPeriodBaseCount;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 4);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eClearTxDoneIrq_Cmd (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eClearTxDoneIrq_Cmd (void) {

	T_eError_eErrorType l_eReturnValue;
	uint16_t l_usIrqMask = 0x4001; /* Tx Done */

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iClearIrqStatusOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)(l_usIrqMask >> 8);
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)l_usIrqMask;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 3);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eClearTxDoneIrq_Cmd (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eClearAllIrqs_Cmd (void) {

	T_eError_eErrorType l_eReturnValue;
	uint16_t l_usIrqMask = 0xFFFF; /* Tx Done */

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iClearIrqStatusOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = (uint8_t)(l_usIrqMask >> 8);
	gs_pucSx1280_iSpiTxBuffer[2] = (uint8_t)l_usIrqMask;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 3);

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eClearTxDoneIrq_Cmd (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eIrqStatus_Get (uint16_t *out_pusIrqStatus) {

	T_eError_eErrorType l_eReturnValue;
	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iGetIrqStatusOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = 0;
	gs_pucSx1280_iSpiTxBuffer[2] = 0;
	gs_pucSx1280_iSpiTxBuffer[3] = 0;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 4);

	*out_pusIrqStatus = 0;
	*out_pusIrqStatus = gs_pucSx1280_iSpiRxBuffer[2];
	*out_pusIrqStatus = ((*out_pusIrqStatus)<<8) + gs_pucSx1280_iSpiRxBuffer[3];

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eClearTxDoneIrq_Cmd (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eRxBufferStatus_Get (uint8_t *out_pucRxPacketLength) {

	T_eError_eErrorType l_eReturnValue;
	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iGetRxBufferStatusOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = 0;
	gs_pucSx1280_iSpiTxBuffer[2] = 0;
	gs_pucSx1280_iSpiTxBuffer[3] = 0;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 4);

	*out_pucRxPacketLength = gs_pucSx1280_iSpiRxBuffer[2];

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eClearTxDoneIrq_Cmd (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eRssiInst_Get (int8_t *out_pcRssiInst) {

	T_eError_eErrorType l_eReturnValue;
	uint8_t l_ucRssiInst;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iGetRssiInstOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = 0;
	gs_pucSx1280_iSpiTxBuffer[2] = 0;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 3);

	l_ucRssiInst = 0 - gs_pucSx1280_iSpiRxBuffer[2] / 2;

	*out_pcRssiInst = (int8_t)l_ucRssiInst;

	return l_eReturnValue;
}


/**
 * \fn T_eError_eErrorType eSx1280_eClearTxDoneIrq_Cmd (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_ePacketStatus_Get (int8_t *out_pcSnr) {

	T_eError_eErrorType l_eReturnValue;
	int8_t l_cSnr;

	gs_pucSx1280_iSpiTxBuffer[0] = D_ucSx1280_iGetPacketStatusOpCode;
	gs_pucSx1280_iSpiTxBuffer[1] = 0;
	gs_pucSx1280_iSpiTxBuffer[2] = 0;
	gs_pucSx1280_iSpiTxBuffer[3] = 0;
	gs_pucSx1280_iSpiTxBuffer[4] = 0;
	gs_pucSx1280_iSpiTxBuffer[5] = 0;
	gs_pucSx1280_iSpiTxBuffer[6] = 0;

	l_eReturnValue = eSx1280_eSendCommand_Cmd(gs_pucSx1280_iSpiTxBuffer, gs_pucSx1280_iSpiRxBuffer, 7);

	l_cSnr = (int8_t)gs_pucSx1280_iSpiRxBuffer[3];

	*out_pcSnr = l_cSnr / 4;

	return l_eReturnValue;
}


static uint32_t gs_uiSx1280_iIsDio1IrqDone = 0;
static uint32_t gs_uiSx1280_iIsDio2IrqDone = 0;


/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eSendData_Cmd (uint8_t *in_pucBuffer, uint8_t in_ucLength,
										   uint32_t in_uiRfFrequencyInKhz, T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor,
										   int8_t in_cTxPower, uint32_t in_uiTxTimeout) {

	T_eError_eErrorType l_eReturnValue;
	uint16_t l_usIrqStatus;
	uint32_t l_uiTimeout;

	l_eReturnValue = eSx1280_eStandbyMode_Set(C_eSx1280_iStandbyRc);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_ePacketType_Set (C_eSx1280_ieLoraMode);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eRfFrequencyInKhz_Set(in_uiRfFrequencyInKhz); /* 2400105 */
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eBufferBaseAddress_Set();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eLoraModulationParams_Set(in_eLoraSpreadingFactor);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}


	l_eReturnValue = eSx1280_ePacketParams_Set(in_ucLength);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eTxPower_Set (in_cTxPower);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eWriteBuffer_Cmd (in_pucBuffer, in_ucLength);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eTxDioIrqParams_Set();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eClearAllIrqs_Cmd();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	HAL_GPIO_WritePin(D_uiGpio_eE28TxEnable_GPIO_Port, D_uiGpio_eE28TxEnable_Pin, GPIO_PIN_SET);

//	osDelay(2);

//	l_eReturnValue = eSx1280_eIrqStatus_Get (&l_usIrqStatus);
//	if(l_eReturnValue != Success) {
//		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
//		return l_eReturnValue;
//	}

	gs_uiSx1280_iIsDio1IrqDone = 0;

	l_eReturnValue = eSx1280_eTx_Set();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_uiTimeout = 0;
	do {
		osDelay(2);
		l_uiTimeout++;
	}while( (gs_uiSx1280_iIsDio1IrqDone == 0) && (l_uiTimeout < (in_uiTxTimeout/2)) );
	if(gs_uiSx1280_iIsDio1IrqDone == 1) {
		eSx1280_eIrqStatus_Get (&l_usIrqStatus);
		gs_uiSx1280_iIsDio1IrqDone = 0;
	}

	HAL_GPIO_WritePin(D_uiGpio_eE28TxEnable_GPIO_Port, D_uiGpio_eE28TxEnable_Pin, GPIO_PIN_RESET);

	l_eReturnValue = eSx1280_eClearAllIrqs_Cmd();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eStandbyMode_Set(C_eSx1280_iStandbyRc);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	return Success;
}



/**
 * \fn T_eError_eErrorType eSx1280_eBufferBaseAddress_Set (void)
 * \brief see sx1280.h for more details on this function.
 */
T_eError_eErrorType eSx1280_eReceiveData_Cmd (uint8_t *out_pucBuffer, uint8_t *out_pucLength,
											  uint32_t in_uiRfFrequencyInKhz, T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor,
											  uint32_t in_uiRxTimeout, T_stSx1280_eRxPacketStatus* out_pstRxPacketStatus) {

	T_eError_eErrorType l_eReturnValue;
	T_eError_eErrorType l_eRxReturnValue;
	uint16_t l_usIrqStatus;
	uint32_t l_uiTimeout;
	uint32_t l_uiIsPreambleDetected;
	uint32_t l_uiNbrRssiSamples;
	int8_t l_cRssiInst;
	int8_t l_cSnr;

	out_pstRxPacketStatus->m_cMinRssiInst = 0;
	out_pstRxPacketStatus->m_cMaxRssiInst = -128;
	out_pstRxPacketStatus->m_cSnr = -128;

	l_eReturnValue = eSx1280_eStandbyMode_Set(C_eSx1280_iStandbyRc);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_ePacketType_Set (C_eSx1280_ieLoraMode);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eRfFrequencyInKhz_Set(in_uiRfFrequencyInKhz);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eBufferBaseAddress_Set();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eLoraModulationParams_Set(in_eLoraSpreadingFactor);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}


	l_eReturnValue = eSx1280_ePacketParams_Set(0);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eRxDioIrqParams_Set();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eClearAllIrqs_Cmd();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	HAL_GPIO_WritePin(D_uiGpio_eE28RxEnable_GPIO_Port, D_uiGpio_eE28RxEnable_Pin, GPIO_PIN_SET);

	gs_uiSx1280_iIsDio1IrqDone = 0;
	gs_uiSx1280_iIsDio2IrqDone = 0;
	l_uiIsPreambleDetected = 0;
	l_uiNbrRssiSamples = 0;

	l_eReturnValue = eSx1280_eRx_Set();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_uiTimeout = 0;
	do {
		osDelay(2);
		l_uiTimeout++;
		if(gs_uiSx1280_iIsDio2IrqDone == 1) {
			l_eReturnValue = eSx1280_eIrqStatus_Get (&l_usIrqStatus);
			if(l_eReturnValue != Success) {
				D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
				return l_eReturnValue;
			}
			if((l_usIrqStatus & D_uiSx1280_iPreambleDetectedIrqMask) == D_uiSx1280_iPreambleDetectedIrqMask) {
				l_uiIsPreambleDetected = 1;
			}
			gs_uiSx1280_iIsDio2IrqDone = 0;
		}
		if( (l_uiIsPreambleDetected == 1) && (l_uiNbrRssiSamples < 16) ) {
			l_eReturnValue = eSx1280_eRssiInst_Get (&l_cRssiInst);
			if(l_eReturnValue != Success) {
				D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
				return l_eReturnValue;
			}
			if(l_cRssiInst < out_pstRxPacketStatus->m_cMinRssiInst) {
				out_pstRxPacketStatus->m_cMinRssiInst = l_cRssiInst;
			}
			if(l_cRssiInst > out_pstRxPacketStatus->m_cMaxRssiInst) {
				out_pstRxPacketStatus->m_cMaxRssiInst = l_cRssiInst;
			}
			l_uiNbrRssiSamples++;
		}
	}while( (gs_uiSx1280_iIsDio1IrqDone == 0) && (l_uiTimeout < (in_uiRxTimeout/2)) );

	l_eReturnValue = eSx1280_eIrqStatus_Get (&l_usIrqStatus);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	if( (l_usIrqStatus & D_uiSx1280_iCrcErrorIrqMask) == D_uiSx1280_iCrcErrorIrqMask) {
		out_pstRxPacketStatus->m_eCrcStatus = C_eSx1280_iBadCrc;
	}
	else {
		out_pstRxPacketStatus->m_eCrcStatus = C_eSx1280_iCrcOk;
	}

	if( (gs_uiSx1280_iIsDio1IrqDone == 1) && ( (l_usIrqStatus & D_uiSx1280_iRxDoneIrqMask) == D_uiSx1280_iRxDoneIrqMask) ) {
		l_eReturnValue = eSx1280_eRxBufferStatus_Get (out_pucLength);
		if(l_eReturnValue != Success) {
			D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
			return l_eReturnValue;
		}

		l_eReturnValue = eSx1280_eReadBuffer_Cmd (out_pucBuffer, *out_pucLength);
		if(l_eReturnValue != Success) {
			D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
			return l_eReturnValue;
		}

		l_eReturnValue = eSx1280_ePacketStatus_Get (&l_cSnr);
		out_pstRxPacketStatus->m_cSnr = l_cSnr;
		if(l_eReturnValue != Success) {
			D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
			return l_eReturnValue;
		}

		l_eRxReturnValue = Success;
	}
	else {
		l_eRxReturnValue = Failure_TimeOut;
	}

	gs_uiSx1280_iIsDio1IrqDone = 0;

	HAL_GPIO_WritePin(D_uiGpio_eE28RxEnable_GPIO_Port, D_uiGpio_eE28RxEnable_Pin, GPIO_PIN_RESET);

	l_eReturnValue = eSx1280_eClearAllIrqs_Cmd();
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	l_eReturnValue = eSx1280_eStandbyMode_Set(C_eSx1280_iStandbyRc);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in Sx1280.c Line %d", __LINE__);
		return l_eReturnValue;
	}

	return l_eRxReturnValue;
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if(GPIO_Pin == D_uiGpio_eE28Dio1_Pin) {
		gs_uiSx1280_iIsDio1IrqDone = 1;
	}

	if(GPIO_Pin == D_uiGpio_eE28Dio2_Pin) {
		gs_uiSx1280_iIsDio2IrqDone = 1;
	}

}

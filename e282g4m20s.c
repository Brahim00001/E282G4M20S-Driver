/**
 * \file e282g4m20s.c
 * \brief E28_2G4M20S Driver Source.
 * \author Brahim Ben Sedrine
 * \version 0.1
 * \date 02 April 2024
 *
 * Source file for E28_2G4M20S driver.
 *
 */

#include "cmsis_os.h"

#include "e282g4m20s.h"
#include "cmd.h"


static uint32_t gs_uiE282G4M20S_iIsDebugEnabled = 0;

/**
 * \fn T_eError_eErrorType eE282G4M20S_eEnableDebug_Cmd (void)
 * \brief see e282g4m20s.h for more details on this function.
 */
T_eError_eErrorType eE282G4M20S_eEnableDebug_Cmd (void) {

	gs_uiE282G4M20S_iIsDebugEnabled = 1;

	return Success;
}

/**
 * \fn T_eError_eErrorType eE282G4M20S_eDisableDebug_Cmd (void)
 * \brief see e282g4m20s.h for more details on this function.
 */
T_eError_eErrorType eE282G4M20S_eDisableDebug_Cmd (void) {

	gs_uiE282G4M20S_iIsDebugEnabled = 0;

	return Success;
}



/**
 * \fn T_eError_eErrorType eE282G4M20S_eEnable_Cmd (void)
 * \brief see e282g4m20s.h for more details on this function.
 */
T_eError_eErrorType eE282G4M20S_eEnable_Cmd (void) {

	osDelay(100);

	/* Power-On E282G4M20S module */
	HAL_GPIO_WritePin(D_uiGpio_eTransceiverEnable_GPIO_Port, D_uiGpio_eTransceiverEnable_Pin, GPIO_PIN_SET);

	osDelay(100);

	/* Exit E282G4M20S module from Reset State */
	HAL_GPIO_WritePin(D_uiGpio_eE28ResetN_GPIO_Port, D_uiGpio_eE28ResetN_Pin, GPIO_PIN_SET);

	osDelay(100);

	return Success;
}


/**
 * \fn T_eError_eErrorType eE282G4M20S_eDisable_Cmd (void)
 * \brief see e282g4m20s.h for more details on this function.
 */
T_eError_eErrorType eE282G4M20S_eDisable_Cmd (void) {

	osDelay(100);

	/* Reset E282G4M20S module */
	HAL_GPIO_WritePin(D_uiGpio_eE28ResetN_GPIO_Port, D_uiGpio_eE28ResetN_Pin, GPIO_PIN_RESET);

	osDelay(100);

	/* Power-Off E282G4M20S module */
	HAL_GPIO_WritePin(D_uiGpio_eTransceiverEnable_GPIO_Port, D_uiGpio_eTransceiverEnable_Pin, GPIO_PIN_RESET);

	osDelay(100);

	return Success;
}


/**
 * \fn T_eError_eErrorType eE282G4M20S_eEnable_Cmd (void)
 * \brief see e282g4m20s.h for more details on this function.
 */
T_eError_eErrorType eE282G4M20S_eSendData_Cmd (uint8_t *in_pucBuffer, uint8_t in_ucLength,
											   uint32_t in_uiRfFrequencyInKhz, T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor,
											   T_eE282G4M20S_eTxPower in_eTxPower, uint32_t in_uiTxTimeout) {

	T_eError_eErrorType l_eReturnValue;

//	l_eReturnValue = eE282G4M20S_eEnable_Cmd();
//	if(l_eReturnValue != Success) {
//		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in e282g4m20s.c Line %d", __LINE__);
//		return l_eReturnValue;
//	}

	l_eReturnValue = eSx1280_eSendData_Cmd (in_pucBuffer, in_ucLength, in_uiRfFrequencyInKhz, in_eLoraSpreadingFactor, in_eTxPower, in_uiTxTimeout);
	if(l_eReturnValue != Success) {
		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in e282g4m20s.c Line %d", __LINE__);
		return l_eReturnValue;
	}

//	l_eReturnValue = eE282G4M20S_eDisable_Cmd();
//	if(l_eReturnValue != Success) {
//		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in e282g4m20s.c Line %d", __LINE__);
//		return l_eReturnValue;
//	}

	return Success;
}


/**
 * \fn T_eError_eErrorType eE282G4M20S_eEnable_Cmd (void)
 * \brief see e282g4m20s.h for more details on this function.
 */
T_eError_eErrorType eE282G4M20S_eReceiveData_Cmd (uint8_t *out_pucBuffer, uint8_t *out_pucLength, int8_t *out_pcRssi,
												  uint32_t in_uiRfFrequencyInKhz, T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor,
												  uint32_t in_uiRxTimeout) {

	T_eError_eErrorType l_eReturnValue;
	T_eError_eErrorType l_eRxReturnValue;
	T_stSx1280_eRxPacketStatus l_stRxPacketStatus;

//	l_eReturnValue = eE282G4M20S_eEnable_Cmd();
//	if(l_eReturnValue != Success) {
//		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in e282g4m20s.c Line %d", __LINE__);
//		return l_eReturnValue;
//	}

	*out_pcRssi = 127;

	l_eReturnValue = eSx1280_eReceiveData_Cmd (out_pucBuffer, out_pucLength, in_uiRfFrequencyInKhz, in_eLoraSpreadingFactor, in_uiRxTimeout, &l_stRxPacketStatus);
	if(l_eReturnValue == Success) {
		if(l_stRxPacketStatus.m_eCrcStatus == C_eSx1280_iCrcOk) {
			*out_pcRssi = l_stRxPacketStatus.m_cMaxRssiInst;
			if(gs_uiE282G4M20S_iIsDebugEnabled) {
				D_vCmd_eDebugPrint(C_eCmd_eDebugInformation, "Packet Received: CRC Ok, Length = %d, RSSI in [ %d , %d ] dBm, SNR = %d dB", *out_pucLength, l_stRxPacketStatus.m_cMinRssiInst, l_stRxPacketStatus.m_cMaxRssiInst, l_stRxPacketStatus.m_cSnr);
			}
			l_eRxReturnValue = Success;
		}
		else {
			D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Packet Received with Bad CRC");
			l_eRxReturnValue = Failure_BadCrc;
		}
	}
	else {
		if(l_eReturnValue == Failure_TimeOut) {
			if(gs_uiE282G4M20S_iIsDebugEnabled) {
				D_vCmd_eDebugPrint(C_eCmd_eDebugWarning, "Rx Timeout");
			}
		}
		else {
			D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Receive Data Returns %d", l_eReturnValue);
		}
		l_eRxReturnValue = l_eReturnValue;
	}

//	l_eReturnValue = eE282G4M20S_eDisable_Cmd();
//	if(l_eReturnValue != Success) {
//		D_vCmd_eDebugPrint(C_eCmd_eDebugError, "Error in e282g4m20s.c Line %d", __LINE__);
//		return l_eReturnValue;
//	}

	return l_eRxReturnValue;
}

/**
 * \file e282g4m20s.h
 * \brief E28_2G4M20S Driver Header.
 * \author Brahim Ben Sedrine
 * \version 0.1
 * \date 02 April 2024
 *
 * Header file for E28_2G4M20S driver.
 *
 */

#ifndef E282G4M20S_H
#define E282G4M20S_H

#include "main.h"
#include "error.h"
#include "sx1280.h"


/**
 *\enum T_eSx1280_eStandbyConfig
 *\brief Standby mode configuration.
 */
typedef enum
{
	C_eE282G4M20S_iLowTxPower = -18,      	/* Low Tx Power */
	C_eE282G4M20S_iHighTxPower = 13,      	/* High Tx Power */
}T_eE282G4M20S_eTxPower;


T_eError_eErrorType eE282G4M20S_eEnableDebug_Cmd (void);
T_eError_eErrorType eE282G4M20S_eDisableDebug_Cmd (void);


T_eError_eErrorType eE282G4M20S_eEnable_Cmd (void);
T_eError_eErrorType eE282G4M20S_eDisable_Cmd (void);

T_eError_eErrorType eE282G4M20S_eSendData_Cmd (uint8_t *in_pucBuffer, uint8_t in_ucLength,
											   uint32_t in_uiRfFrequencyInKhz, T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor,
											   T_eE282G4M20S_eTxPower in_eTxPower, uint32_t in_uiTxTimeout);
T_eError_eErrorType eE282G4M20S_eReceiveData_Cmd (uint8_t *out_pucBuffer, uint8_t *out_pucLength, int8_t *out_pcRssi,
												  uint32_t in_uiRfFrequencyInKhz, T_eSx1280_eLoraSpreadingFactor in_eLoraSpreadingFactor,
												  uint32_t in_uiRxTimeout);



#endif // E282G4M20S_H

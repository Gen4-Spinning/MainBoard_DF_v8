/*
 * EepromFns.c
 *
 *  Created on: 06-Mar-2023
 *      Author: harsha
 */

#include "Eeprom.h"
#include "machineSettings.h"
#include "machineEepromSettings.h"

void ReadMachineSettingsFromEeprom(machineSettingsTypeDef *m)
{
	m->delivery_mMin = EE_ReadInteger(DF_DELIVERY_M_MIN_ADDR);
	m->draft  = EE_ReadFloat(DF_DRAFT_ADDR);
	m->lengthLimit_m = EE_ReadInteger(DF_LENGTH_LIMIT_M);
	m->rampUpTime = EE_ReadInteger(RAMPUP_TIME_ADDR);
	m->rampDownTime = EE_ReadInteger(RAMPDOWN_TIME_ADDR);
	m->creelTensionFactor = EE_ReadFloat(CREEL_TENSION_ADDR);
}

void ReadALSettingsFromEeprom(ALSettings_BT_TypeDef *al)
{
    al->Kp            = EE_ReadFloat(AL_KP_ADDR);
    al->sliver_nplus1       = EE_ReadInteger(AL_SLIVER_NPLUS1_ADDR);
    al->sliver_n      = EE_ReadInteger(AL_SLIVER_N_ADDR);
    al->sliver_nminus1       = EE_ReadInteger(AL_SLIVER_NMINUS1_ADDR);
    al->target_g_per_m = EE_ReadFloat(AL_TARGET_G_ADDR);
}

uint8_t WriteALSettingsIntoEeprom(ALSettings_BT_TypeDef *al)
{
    uint8_t dataWritten = 0;
    dataWritten += EE_WriteFloat(al->Kp, AL_KP_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(al->sliver_nplus1, AL_SLIVER_NPLUS1_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(al->sliver_n, AL_SLIVER_N_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(al->sliver_nminus1, AL_SLIVER_NMINUS1_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteFloat(al->target_g_per_m, AL_TARGET_G_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(AL_VALID_FLAG_VALUE, AL_VALID_FLAG_ADDR);

    return (dataWritten == 6) ? 0 : 1;  // 0 = success, same as existing pattern
}

uint8_t CheckALSettings(ALSettings_BT_TypeDef *al)
{
    if (al->Kp <= 0.0f || al->Kp > 1.0f)           return 0;
    if (al->sliver_nplus1 >= al->sliver_n)                 return 0;
    if (al->sliver_n >= al->sliver_nminus1)                 return 0;
    if (al->sliver_nplus1 < 100 || al->sliver_nplus1 > 2000)    return 0;
    if (al->sliver_nminus1 < 100 || al->sliver_nminus1 > 2000)    return 0;
    if (al->target_g_per_m <= 0.0f || al->target_g_per_m > 50.0f) return 0;
    return 1;
}

void LoadDefaultALSettings(ALSettings_BT_TypeDef *al)
{
    al->Kp             = DEFAULT_AL_KP;
    al->sliver_nplus1        = DEFAULT_AL_SLIVER6;
    al->sliver_n        = DEFAULT_AL_SLIVER5;
    al->sliver_nminus1        = DEFAULT_AL_SLIVER4;
    al->target_g_per_m = DEFAULT_AL_TARGET_G;
}
uint8_t WriteMachineSettingsIntoEeprom(machineSettingsTypeDef *m)
{
	uint8_t dataWritten = 0;
    dataWritten += EE_WriteInteger(m->delivery_mMin,DF_DELIVERY_M_MIN_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteFloat(m->draft,DF_DRAFT_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(m->lengthLimit_m,DF_LENGTH_LIMIT_M);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(m->rampUpTime,RAMPUP_TIME_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(m->rampDownTime,RAMPDOWN_TIME_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteFloat(m->creelTensionFactor,CREEL_TENSION_ADDR);
    if (dataWritten == 6)
    	{return 0;}
    else{
    	return 1;}

}


uint8_t CheckMachineSettings(machineSettingsTypeDef* m){
	//typically when something goes wrong with the eeprom you get a value that is very high..
	//to allow for changes place to place without changing this code, we just set the thresholds to  2* maxRange.
	// dont expect in any place the nos to go higher than this..NEED TO PUT LOWER BOUNDS FOR EVERYTHING
	if ((m->delivery_mMin > 150 ) || (m->delivery_mMin < 50)){
		return 0;
	}
	if ((m->draft > 12.0f)||(m->draft < 3)){
		return 0;
	}
	if ((m->lengthLimit_m > 1250)||(m->lengthLimit_m < 5)){
		return 0;
	}

	if ((m->rampUpTime > 15) || (m->rampUpTime < 1)){
		return 0;
	}
	if ((m->rampDownTime > 15) || (m->rampDownTime < 1)){
		return 0;
	}
	if ((m->creelTensionFactor > 5) || (m->creelTensionFactor < 0.5)){
		return 0;
	}

	return 1;
}

void LoadDefaultMachineSettings(machineSettingsTypeDef* m){
	m->delivery_mMin = DEFAULT_DELIVERY_M_MIN;
	m->draft = DEFAULT_DRAFT;
	m->lengthLimit_m = DEFAULT_LENGTHLIMIT;
	m->rampUpTime = DEFAULT_RAMPUP_TIME;
	m->rampDownTime = DEFAULT_RAMPDOWN_TIME;
	m->creelTensionFactor = DEFAULT_CREELTENSION_FACTOR;
}

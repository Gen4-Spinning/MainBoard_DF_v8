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
	if ((m->draft > 12.0f)||(m->draft < 5)){
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

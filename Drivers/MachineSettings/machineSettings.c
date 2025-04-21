/*
 * machineSettings.c
 *
 *  Created on: Apr 11, 2023
 *      Author: harsha
 */

#include "machineSettings.h"

void InitializeMachineSettings(machineSettingsTypeDef *ms){
    ms->delivery_mMin = 80;
    ms->draft = 8;
    ms->lengthLimit_m = 1.1;
    ms->rampUpTime = 5;
    ms->rampDownTime = 5;
    ms->creelTensionFactor = 1;
}

void InitializePiecingSettings(machineSettingsTypeDef *ms){
    ms->delivery_mMin = PIECING_DELIVERY_M_MIN;
    ms->draft = 8;
    ms->lengthLimit_m = 1.1;
    ms->rampUpTime = PIECING_RAMPUP_TIME;
    ms->rampDownTime = PIECING_RAMPDOWN_TIME;
    ms->creelTensionFactor = 1;
}

float CalculateDeliverySpeed(uint16_t FR_rpm){
	float FR_circumference = FR_CIRCUMFERENCE;
	float d_mm_min = FR_rpm * FR_circumference;
	float delivery_m_min = d_mm_min/1000.0f;
	return delivery_m_min;
}

void CalculateMachineParameters(machineSettingsTypeDef *ms,machineParamsTypeDef *mp){
	float FR_circumference = FR_CIRCUMFERENCE;
	mp->FR_RPM = ms->delivery_mMin * 1000.0f/FR_circumference;
	mp->FR_MotorRPM = mp->FR_RPM * FR_TO_FRMOTOR_GEAR_RATIO;
	mp->FR_SurfaceSpeed = mp->FR_MotorRPM/60.0f * FR_circumference;

	mp->reqDraft_FR_TO_SR = ms->draft/BR_TO_SR_BREAK_DRAFT;
	mp->SR_SurfaceSpeed = mp->FR_SurfaceSpeed/mp->reqDraft_FR_TO_SR;
	mp->req_BR_SurfaceSpeed = mp->SR_SurfaceSpeed/BR_TO_SR_BREAK_DRAFT;
	mp->BR_RPM = mp->req_BR_SurfaceSpeed*60.0f/(3.14f * BR_DIA_MM);
	mp->BR_MotorRPM = mp->BR_RPM * BR_TO_BRMOTOR_GEAR_RATIO;

	mp->req_Creel_SurfaceSpeed = mp->req_BR_SurfaceSpeed;
	mp->usedCreelDia = (CREEL_PULLEY_DIA + SLIVER_WIDTH) / ms->creelTensionFactor;
	float creelCirc = mp->usedCreelDia * 3.14f;
	float creelPulleyRPM = mp->req_Creel_SurfaceSpeed*60.0f/creelCirc;
	mp->Creel_MotorRPM = creelPulleyRPM/CREEL_DRIVEPULLEY_TO_MOTOR_RATIO;

}

void Reset_CurrentMtrsRun(machineParamsTypeDef *mp){
	mp->currentMtrsRun = 0;
}



uint8_t getMotorCANAddress(uint8_t motor){
	if (motor == FR){
		return 2;
	}else if (motor == BR){
		return 3;
	}else if (motor == CREEL){
		return 4;
	}else{
		return 0;
	}
}

uint8_t GetMotorID_from_CANAddress(uint8_t canAddress){
	if (canAddress == FRONTROLLER_ADDRESS){
		return FR;
	}else if (canAddress == BACKROLLER_ADDRESS){
		return BR;
	}else if (canAddress == CREEL_ADDRESS){
		return CREEL;
	}
	return 6; // return
}

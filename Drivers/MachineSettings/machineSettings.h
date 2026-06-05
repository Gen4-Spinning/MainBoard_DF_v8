/*
 * machineSettings.h
 *
 *  Created on: Apr 11, 2023
 *      Author: harsha
 */

#ifndef MACHINESETTINGS_H_
#define MACHINESETTINGS_H_

#include "stdio.h"
#include "FDCAN.h"


//all Structs with arrays have to follow this Form
#define FR 0
#define BR 1
#define CREEL 2

#define FR_DIA_MM 40
#define BR_DIA_MM 30
#define FR_TO_FRMOTOR_GEAR_RATIO 1.0f
#define BR_TO_BRMOTOR_GEAR_RATIO 3.0f//3.07
#define BR_TO_SR_BREAK_DRAFT 1.48f
#define CREEL_DRIVEPULLEY_TO_MOTOR_RATIO 0.722f
#define CREEL_PULLEY_DIA 54
#define SLIVER_WIDTH 14

#define FR_CIRCUMFERENCE FR_DIA_MM * 3.14f

typedef struct machineSettings_Struct{
    int delivery_mMin;
    float draft;
    int lengthLimit_m;
    int rampUpTime;
    int rampDownTime;
    float creelTensionFactor;

}machineSettingsTypeDef;

typedef struct machinePidSettings{
	uint8_t motorID;
	uint16_t Kp;
	uint16_t Ki;
	uint16_t FF;
	uint16_t SO;
}mcPIDSettings;

typedef struct AutoLeveller{
	float FR_Surface_Speed;
	float Draft;
}ALcalculation;


typedef struct machineParamaters_Struct{
    uint16_t FR_MotorRPM;
    uint16_t BR_MotorRPM;
    uint16_t Creel_MotorRPM;

    float reqDraft_FR_TO_SR;

    float FR_SurfaceSpeed;
    float SR_SurfaceSpeed;
    float req_BR_SurfaceSpeed;
    float req_Creel_SurfaceSpeed;

    float usedCreelDia;

    float FR_RPM;
	float BR_RPM;

	float currentDeliveryMMin;
	float currentMtrsRun;
	float totalPower;
}machineParamsTypeDef;
// ADD this typedef inside machineSettings.h
// place it after the existing machineSettingsTypeDef struct

typedef struct {
    float    Kp;
    uint16_t sliver_nplus1;
    uint16_t sliver_n;
    uint16_t sliver_nminus1;
    float    target_g_per_m;
} ALSettings_BT_TypeDef;

extern ALSettings_BT_TypeDef al_bt;
extern ALcalculation AL;
extern machineSettingsTypeDef msp;
extern machineSettingsTypeDef ps;
extern machineSettingsTypeDef msp_BT;
extern machineParamsTypeDef mcParams;
extern mcPIDSettings pid_BT;

// Eeprom MachineSettings
void ReadMachineSettingsFromEeprom(machineSettingsTypeDef *m);
uint8_t WriteMachineSettingsIntoEeprom(machineSettingsTypeDef *m);
uint8_t CheckMachineSettings(machineSettingsTypeDef* m);
void LoadDefaultMachineSettings(machineSettingsTypeDef* m);

//otherFns
void InitializeMachineSettings(machineSettingsTypeDef *ms);
void CalculateMachineParameters(machineSettingsTypeDef *ms,machineParamsTypeDef *mp);
float CalculateDeliverySpeed(uint16_t FR_rpm);
void Reset_CurrentMtrsRun(machineParamsTypeDef *mp);

uint8_t getMotorCANAddress(uint8_t motor);
uint8_t GetMotorID_from_CANAddress(uint8_t canAddress);

void InitializePiecingSettings(machineSettingsTypeDef *ms);

#endif /* MACHINESETTINGS_H_ */

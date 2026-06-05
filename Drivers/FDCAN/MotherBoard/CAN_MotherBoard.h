/*
 * CAN_MotherBoard.h
 *
 *  Created on: Mar 7, 2023
 *      Author: harsha
 */

#ifndef CAN_MOTHERBOARD_H_
#define CAN_MOTHERBOARD_H_
#include "main.h"
#include "stm32g4xx_hal.h"
#include "stdio.h"
#include "Struct.h"
#include "MotorComms.h"
#include "FDCAN.h"
#include "CommonConstants.h"
#include "Struct.h"
#include "StateMachine.h"
#include "machineSettings.h"
#include "MachineErrors.h"
#include "SysObserver.h"
#include "Log.h"
#include "Ack.h"
#include "DataRequest.h"
/* ---------------------------------------------------------------
 * ADD to:  MainBoard_DF_v8/Drivers/FDCAN/MotherBoard/CAN_MotherBoard.h
 * --------------------------------------------------------------- */

/* AL settings frame ACK status */
#define AL_SETTINGS_ACK_PENDING  0
#define AL_SETTINGS_ACK_OK       1
#define AL_SETTINGS_ACK_FAIL     2

typedef struct {
    uint8_t  ackStatus;          /* AL_SETTINGS_ACK_PENDING/OK/FAIL  */
    uint8_t  retryCnt;
    uint32_t sentTimestamp;      /* HAL_GetTick() at send time        */
} ALSettingsSendState_TypeDef;

extern ALSettingsSendState_TypeDef ALSendState;

/* Send AL settings (KP, Sliver thresholds, TARGET_G) to BR motor  */
void FDCAN_SendALSettings_ToMotor(uint8_t destination,
                                  float kp,
                                  uint16_t sliver6,
                                  uint16_t sliver5,
                                  uint16_t sliver4,
                                  float target_g_per_m);

/* Called from parser when ACK for AL settings is received         */
void FDCAN_Receive_ALSettingsACK(void);

/* Called from parser when sensor toggle frame arrives             */
void FDCAN_Receive_SensorToggle(void);

/* Retry logic – call from a periodic task (e.g. 100 ms timer)    */
void FDCAN_ALSettings_CheckACK(uint8_t destination,
                                float kp,
                                uint16_t sliver6,
                                uint16_t sliver5,
                                uint16_t sliver4,
                                float target_g_per_m);
void FDCAN_sendCommand_ToMotor(uint8_t destination,uint8_t command);
void FDCAN_sendSetUp_ToMotor(uint8_t destination, SetupMotor rd);
void FDCAN_SendDiagnostics_ToMotor(uint8_t destination,DiagnosticsTypeDef *d);
void FDCAN_sendChangeTarget_ToMotor(uint8_t destination, uint16_t newTarget, uint16_t transitionTime);
void FDCAN_sendConsoleCHK_Response(uint8_t destination);
void FDCAN_sendDataRequest_ToMotor(uint8_t destination,uint8_t requestType);
void FDCAN_sendPID_Update_toMotor(uint8_t destination);
void FDCAN_SendMachineCalculationTOBR_AL(void);


void FDCAN_Recieve_ACKFromMotors(uint8_t sourceAddress);
void FDCAN_Recieve_RunDataFromMotors(uint8_t source);
void FDCAN_Recieve_ErrorsFromMotors(uint8_t sourceAddress);

void FDCAN_parseForMotherBoard(void);

#endif /* CAN_MOTHERBOARD_H_ */

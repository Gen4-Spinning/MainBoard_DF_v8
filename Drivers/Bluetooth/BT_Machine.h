/*
 * BT_Constants.h
 *
 *  Created on: Mar 9, 2023
 *      Author: harsha
 */

#ifndef BT_MC_H_
#define BT_MC_H_

#include "BT_Fns.h"
#include "machineSettings.h"

//settings TLVs
#define DELIVERY_M_MIN_BT 0x70
#define DRAFT_BT 0x71
#define LENGTH_LIMIT_M_BT 0x72
#define RAMPUP_BT 0x73
#define RAMPDOWN_BT 0x74
#define CREEL_TENSION_FACTOR_BT 0x75

#define BT_FR 0x01
#define BT_BR 0x02
#define BT_CREEL 0x03
#define BT_DRAFTING 0x05

#define BT_DIAGNOSTIC_RUN 0x01
#define BT_DIAGNOSTIC_STOP 0x06


uint8_t BT_MC_generateSettingsMsg(machineSettingsTypeDef *m);
uint8_t BT_MC_generatePIDValuesMsg(uint16_t Kp,uint16_t Ki,uint16_t FF, uint16_t SO);
uint8_t BT_MC_parse_Settings(machineSettingsTypeDef *mspBT);
uint8_t BT_MC_Save_Settings(void);
uint8_t GetMotorID_from_BTMotor_ID(uint8_t BT_motorID);
uint8_t GetMotorId_from_CarousalID(uint8_t carousalID);
uint8_t Get_BTMotorID_from_Motor_ID(uint8_t motorID);
uint8_t BT_PID_parse_Request(mcPIDSettings *pBT);
uint8_t BT_PID_parse_NewSettings(mcPIDSettings *pBT);

#endif /* BT_MC_H_ */

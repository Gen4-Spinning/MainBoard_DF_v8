/*
 * FDCAN.h
 *
 *  Created on: Mar 5, 2023
 *      Author: Jonathan
 */

#ifndef FDCAN_FDCAN_H_
#define FDCAN_FDCAN_H_

#include "stm32g4xx_hal.h"

#define MOTHERBOARD_ADDRESS		0x01
#define FRONTROLLER_ADDRESS 	0x02
#define BACKROLLER_ADDRESS	 	0x03
#define CREEL_ADDRESS 			0x04
#define AUTOLEVELLER_ADDRESS	0X0A

#define MOTORSTATE_FUNCTIONID			0x01
#define ERROR_FUNCTIONID				0x02

#define DATA_REQUEST_RESPONSE 			0x05 // after we make a request , the response ID

#define RUNSETUPDATA_FUNCTIONID			0x07
#define ANALYSISDATA_FUNCTIONID			0x08
#define RUNTIMEDATA_FUNCTIONID			0x09
#define DIAGNOSTICSDATA_FUNCTIONID 		0x0A

#define CHANGETARGET_FUNCTIONID			0x0D
#define ACKFRAME_FUNCTIONID				0x0F
#define DIAGNOSTICSDONEFRAME_FUNCTIONID	0X14

#define CYLINDEREXTENDEDDATA_FUNCTIONID 0x0B
#define DRIVE_CAN_CHK_REQUEST 0x18
#define PID_UPDATE_RESPONSE   0x1B

#define ACK_FROM_BR_CALCULATIONS  0x20

#define PRIORITY1 0x06//priority1 has the highest priority
#define PRIORITY2 0x0A
#define PRIORITY3 0x0E


#define MOTHERBOARD_RECIEVE_MASK 0x00000100 //this is the value you want in the mask
#define MOTHERBOARD_RECIEVE_FILTER 0x0000FF00 //this is the position you want to mask
#define AL_SETTINGS_FUNCTIONID          0x24

/* Motor → MainBoard: sensor toggle switch state                   */
#define SENSOR_TOGGLE_FUNCTIONID        0x26

/* ACK timeout (ms) for AL settings send */
#define AL_SETTINGS_ACK_TIMEOUT_MS      500

#define AL_CALIB_FUNCTIONID      0x27   // mainboard → motor: start/stop calibration
#define AL_CALIB_RESULT_FUNCTIONID 0x28 // motor → mainboard: average result

#define AL_CALIB_FUNCTIONID         0x27   // MB → motor: start/stop collecting
#define AL_CALIB_RESULT_FUNCTIONID  0x28   // motor → MB: average result
void FDCAN_TxInit(void);
void FDCAN_RxFilterInit(void);
uint32_t FDCAN_generateIdentifier(uint16_t source, uint16_t destination, uint16_t functionID, uint8_t priority);

//variables that you want else where define as extern here
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_RxHeaderTypeDef   RxHeader;
extern FDCAN_TxHeaderTypeDef   TxHeader;

extern uint8_t TxData[16];
extern uint8_t RxData[32];
extern uint32_t functionID;
extern uint32_t source_address;


#endif /* FDCAN_FDCAN_H_ */

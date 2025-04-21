/*
 * SettingsState.c
 *
 *  Created on: Mar 10, 2023
 *      Author: harsha
 */

#include "Struct.h"
#include "BT_Machine.h"
#include "MachineErrors.h"
#include "CommonConstants.h"
#include "stm32g4xx_hal.h"
#include "StateMachine.h"
#include "DataRequest.h"

extern UART_HandleTypeDef huart1;

void SettingsState(void){

	uint8_t success = 0;
	uint8_t PIDmotorNo = 0;
	uint8_t packetSize = 0;
	if (BT.information == REQ_SETTINGS_FROM_APP){
		  packetSize = BT_MC_generateSettingsMsg(&msp);
		  while(S.BT_transmission_over != 1){};
		  HAL_UART_Transmit_IT(&huart1,(uint8_t*)BufferTransmit,packetSize);
		  S.BT_transmission_over = 0;
	}

	else if (BT.information == SETTINGS_FROM_APP){
	  if (S.BT_dataOK){
		  success = BT_MC_Save_Settings();
		  if (success == 0){
			MBE.AppSettings_eepromWriteFailure += 1;
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGFAILURE,6);
			S.BT_transmission_over = 1;
		 }else{
			 // send success msg to App
			 HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGSUCCESS,6);
			 S.BT_transmission_over = 1;
		 }
		  S.BT_dataOK = 0;
	  }else{
		  S.BT_transmission_over = 0;
		  MBE.AppSettings_parsingFailure += 1;
		  HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGFAILURE,6);
	  }
	}


	else if (BT.information == PID_REQUEST){
		if (S.BT_dataOK){
			PIDmotorNo = GetMotorID_from_BTMotor_ID(pid_BT.motorID);
			SendDataRequest(&DR,PID_SETTINGS_REQUEST,PIDmotorNo);
			//wait till response is recieved or timed out - 2sec timeout
			while((DR.timer < DATA_REQUEST_TIMEOUT_SEC) && (DR.responseRecieved == 0)){}
			if (DR.responseRecieved == 0){
				while(S.BT_transmission_over != 1){};
				HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGFAILURE,6);
				S.BT_transmission_over = 0;
			}else{
				packetSize = BT_MC_generatePIDValuesMsg(DR.p.Kp_motorID,DR.p.Ki_motorID,DR.p.FF_motorID,DR.p.startOffset_motorID);
				while(S.BT_transmission_over != 1){};
				HAL_UART_Transmit_IT(&huart1,(uint8_t*)BufferTransmit,packetSize);
				S.BT_transmission_over = 0;
			}
			DR.requestSent = 0;
			DR.requestType = 0;
			DR.motorName = 0;
			DR.timer = 0;
		}else{
			while(S.BT_transmission_over != 1){};
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGFAILURE,6);
			S.BT_transmission_over = 0;
		}
		DR.responseRecieved = 0;
	}

	else if (BT.information == PID_NEW_VALS){
		if (S.BT_dataOK){
			DR.p.motorID = GetMotorID_from_BTMotor_ID(pid_BT.motorID);
			DR.p.Kp_motorID = pid_BT.Kp;
			DR.p.Ki_motorID = pid_BT.Ki;
			DR.p.FF_motorID = pid_BT.FF;
			DR.p.startOffset_motorID = pid_BT.SO;
			SendDataRequest(&DR,PID_SETTINGS_UPDATE,DR.p.motorID);
			//wait till response is recieved or timed out - 2sec timeout
			while((DR.timer < DATA_REQUEST_TIMEOUT_SEC) && (DR.responseRecieved == 0)){}
			if (DR.responseRecieved == 1){
				if (DR.responseSuccess == 1){
					while(S.BT_transmission_over != 1){};
					HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGSUCCESS,6);
					S.BT_transmission_over = 0;
				}else{
					while(S.BT_transmission_over != 1){};
					HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGFAILURE,6);
					S.BT_transmission_over = 0;
				}
			}else{
				while(S.BT_transmission_over != 1){};
				HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGFAILURE,6);
				S.BT_transmission_over = 0;
			}
			DR.requestSent = 0;
			DR.requestType = 0;
			DR.motorName = 0;
			DR.timer = 0;
		}else{
			 S.BT_transmission_over = 0;
			 HAL_UART_Transmit_IT(&huart1,(uint8_t*)SAVINGFAILURE,6);
		}
		DR.responseRecieved = 0;
	}
	else{}
	/* go back to the idle state. the new values get integrated into the
	* machine parameters struct in the idle state when you first enter it
	*/
	HAL_Delay(10);
	ChangeState(&S,S.prev_state);
	if (S.current_state == RUN_STATE){ // if you go back to run state, then dont send the rampUp msg to the cylinders.
	  S.oneTime = 0;
	}
}

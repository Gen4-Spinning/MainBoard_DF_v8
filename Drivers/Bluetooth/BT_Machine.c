/*
 * BT_Machine.c
 *
 *  Created on: 25-Apr-2023
 *      Author: harsha
 */

#include "BT_Machine.h"
#include "CAN_MotherBoard.h"
#include "StateMachine.h"
#include "machineEepromSettings.h"   // for AL_VALID_FLAG_ADDR etc


// if not already present — al_bt is defined in main.c as extern
extern ALSettings_BT_TypeDef al_bt;
uint8_t BT_MC_generateSettingsMsg(machineSettingsTypeDef *m){
	  char TLV_Buffer[12];
	  uint8_t tlvSize = 0;
	  uint8_t eof_size  = 0;
	  uint8_t initLength = 0;

	  initLength = Init_TXBuf_Frame(SETTINGS_FROM_MC,SUBSTATE_NA,6);

	  generateTLV_I(TLV_Buffer,DELIVERY_M_MIN_BT,m->delivery_mMin);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  generateTLV_F(TLV_Buffer,DRAFT_BT,m->draft);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_FLOAT,initLength+tlvSize);
	  tlvSize += TLV_FLOAT;

	  generateTLV_I(TLV_Buffer,LENGTH_LIMIT_M_BT,m->lengthLimit_m);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  generateTLV_I(TLV_Buffer,RAMPUP_BT,m->rampUpTime);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  generateTLV_F(TLV_Buffer,CREEL_TENSION_FACTOR_BT,m->creelTensionFactor);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_FLOAT,initLength+tlvSize);
	  tlvSize += TLV_FLOAT;

	  generateTLV_I(TLV_Buffer,RAMPDOWN_BT,m->rampDownTime);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  eof_size = addEOF(initLength+tlvSize);
	  correctLengthInFrame(initLength,tlvSize,eof_size);

	  return initLength + tlvSize + eof_size;

}

uint8_t BT_MC_generatePIDValuesMsg(uint16_t Kp,uint16_t Ki,uint16_t FF, uint16_t SO){
		char TLV_Buffer[12];
		uint8_t tlvSize = 0;
		uint8_t eof_size  = 0;
		uint8_t initLength = 0;

		initLength = Init_TXBuf_Frame(PID_RESPONSE_TO_REQUEST,SUBSTATE_NA,4);

		generateTLV_I(TLV_Buffer,KP_TYPE,Kp);
		add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
		tlvSize += TLV_INT;

		generateTLV_I(TLV_Buffer,KI_TYPE,Ki);
		add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
		tlvSize += TLV_INT;

		generateTLV_I(TLV_Buffer,FF_TYPE,FF);
		add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
		tlvSize += TLV_INT;

		generateTLV_I(TLV_Buffer,SO_TYPE,SO);
		add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
		tlvSize += TLV_INT;

		eof_size = addEOF(initLength+tlvSize);
		correctLengthInFrame(initLength,tlvSize,eof_size);

		return initLength + tlvSize + eof_size;

}

uint8_t BT_PID_parse_Request(mcPIDSettings *pBT){
	TLVStruct_TypeDef T;
	uint8_t TLV_start = 10;
	uint8_t tlvSize = 0;
	uint8_t count = 0;
	for (int i=0;i<BT.attributeCnt;i++){
		tlvSize = parseTLV(&T,TLV_start);
		TLV_start += tlvSize;
		switch (T.type){
			case PID_REQUEST_WHICH_MOTOR:
				pBT->motorID = T.value_int;
				count += 1;
				break;
		}
	}
	return count; // 1 if ok, 0 if not
}

uint8_t BT_PID_parse_NewSettings(mcPIDSettings *pBT){
	//Buffer Rec index 10 onwards is TLVs till 10 + TlvsLength
	TLVStruct_TypeDef T;
	uint8_t TLV_start = 10;
	uint8_t tlvSize = 0;
	uint8_t count = 0;
	uint8_t allSettingsRecieved = 0;
    for (int i=0;i<BT.attributeCnt;i++){
    	tlvSize = parseTLV(&T,TLV_start);
    	TLV_start += tlvSize;
    	switch (T.type){
    		case PID_REQUEST_WHICH_MOTOR:
    			pBT->motorID = (uint8_t)T.value_int;
    			count += 1;
    			break;
    		case KP_TYPE:
    			pBT->Kp = T.value_int;
    			count += 1;
    			break;
    		case KI_TYPE:
    			pBT->Ki = T.value_int;
    			count += 1;
    			break;
    		case FF_TYPE:
    			pBT->FF = T.value_int;
    			count += 1;
    			break;
    		case SO_TYPE:
    			pBT->SO = T.value_int;
    			count += 1;
    			break;
    	}
    }
    if (count == 5){
    	allSettingsRecieved = 1;
    }

    return allSettingsRecieved;
}

uint8_t BT_MC_parse_Settings(machineSettingsTypeDef *mspBT){
	//Buffer Rec index 10 onwards is TLVs till 10 + TlvsLength
	TLVStruct_TypeDef T;
	uint8_t TLV_start = 10;
	uint8_t tlvSize = 0;
	uint8_t count = 0;
	uint8_t allSettingsRecieved = 0;

    for (int i=0;i<BT.attributeCnt;i++){
    	tlvSize = parseTLV(&T,TLV_start);
    	TLV_start += tlvSize;
    	switch (T.type){
    		case DELIVERY_M_MIN_BT:
    			mspBT->delivery_mMin = T.value_int;
    			count += 1;
    			break;
    		case DRAFT_BT:
    			mspBT->draft = T.value_f;
    			count += 1;
    			break;
    		case LENGTH_LIMIT_M_BT:
    			mspBT->lengthLimit_m = T.value_int;
    			count += 1;
    			break;
    		case RAMPUP_BT:
    			mspBT->rampUpTime = T.value_int;
    			count += 1;
    			break;
    		case RAMPDOWN_BT:
    			mspBT->rampDownTime = T.value_int;
    			count += 1;
    			break;
    		case CREEL_TENSION_FACTOR_BT:
    			mspBT->creelTensionFactor = T.value_f;
    			count += 1;
    			break;
    	}
    }
    if (count == 6){
    	allSettingsRecieved = 1;
    }

    return allSettingsRecieved;
}

//FLYER
uint8_t BT_MC_Save_Settings(void){
	uint8_t fail;
	fail = WriteMachineSettingsIntoEeprom(&msp_BT);
	if (fail == 0){
		msp.delivery_mMin = msp_BT.delivery_mMin;
		msp.draft = msp_BT.draft;
		msp.lengthLimit_m = msp_BT.lengthLimit_m;
		msp.rampDownTime = msp_BT.rampDownTime;
		msp.rampUpTime = msp_BT.rampUpTime;
		msp.creelTensionFactor = msp_BT.creelTensionFactor;
		//send success msg to APP
	}
	return !fail;
}


uint8_t GetMotorID_from_BTMotor_ID(uint8_t BT_motorID){
	if (BT_motorID == BT_FR){
		return FR;
	}else if (BT_motorID == BT_BR){
		return BR;
	}else if (BT_motorID == BT_CREEL){
		return CREEL;
	}
	return 0;
}

uint8_t Get_BTMotorID_from_Motor_ID(uint8_t motorID){
	if (motorID == FR){
		return BT_FR;
	}else if (motorID == BR){
		return BT_BR;
	}else if (motorID == CREEL){
		return BT_CREEL;
	}
	return 0;
}


uint8_t GetMotorId_from_CarousalID(uint8_t carousalID){
	if (carousalID == BT_FR){
		return FR;
	}else if (carousalID == BT_BR){
		return BR;
	}
	else if (carousalID == BT_CREEL){
		return CREEL;
	}
	return 99;
}

/* ------- (1) Add toggle TLV inside BT_MC_generateStatusMsg() ---
 *
 * In the  else if (state == BT_RUN)  block, append:
 *
 *   generateTLV_I(TLV_Buffer, RUN_SENSOR_TOGGLE_STATE, S.sensorToggleState);
 *   add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
 *   tlvSize += TLV_INT;
 *   runAttributes++;   // increment the attribute count declared above
 *
 * Also reset the flag after sending:
 *   S.sensorToggleUpdated = 0;
 * ---------------------------------------------------------------- */


/* ------- (2) New function: parse AL settings from app ---------- */
/*
 * BT_MC_parse_ALSettings()
 *
 * Called when BT.information == AL_SETTINGS_FROM_APP (0x11).
 * Parses TLV payload, stores values, then triggers CAN send.
 *
 * Returns 1 if all 5 TLVs found and valid, else 0.
 */
uint8_t BT_MC_parse_ALSettings(void)
{
	   TLVStruct_TypeDef T;
	      uint8_t location = 10;
	      uint8_t found = 0;
	      float    bt_kp      = 0.0f;
	      uint16_t bt_sliver6 = 0, bt_sliver5 = 0, bt_sliver4 = 0;
	      float    bt_target  = 0.0f;

	      for (uint8_t i = 0; i < BT.attributeCnt; i++) {
	          uint8_t sz = parseTLV(&T, location);
	          if (sz == 0) break;
	          switch (T.type) {
	              case AL_KP_TYPE:       bt_kp      = T.value_f;   found |= 0x01; break;
	              case AL_SLIVER6_TYPE:  bt_sliver6 = T.value_int; found |= 0x02; break;
	              case AL_SLIVER5_TYPE:  bt_sliver5 = T.value_int; found |= 0x04; break;
	              case AL_SLIVER4_TYPE:  bt_sliver4 = T.value_int; found |= 0x08; break;
	              case AL_TARGET_G_TYPE: bt_target  = T.value_f;   found |= 0x10; break;
	          }
	          location += sz;
	      }

	      if (found != 0x1F)                         return 0;
	      if (bt_kp <= 0.0f || bt_kp > 1.0f)        return 0;
	      if (bt_sliver6 >= bt_sliver5)              return 0;
	      if (bt_sliver5 >= bt_sliver4)              return 0;
	      if (bt_target <= 4.0f || bt_target > 6.0f) return 0;

	      /* Store into global — SettingsState will do EEPROM + CAN */
	      al_bt.Kp             = bt_kp;
	      al_bt.sliver_nplus1        = bt_sliver6;
	      al_bt.sliver_n        = bt_sliver5;
	      al_bt.sliver_nminus1        = bt_sliver4;
	      al_bt.target_g_per_m = bt_target;
	      return 1;
}
uint8_t BT_MC_generateCalibResultMsg(void)
{
    char TLV_Buffer[12];
    uint8_t tlvSize = 0, eof_size = 0;
    uint8_t initLength = Init_TXBuf_Frame(AL_CALIB_DATA_FROM_MB, SUBSTATE_NA, 2);

    generateTLV_I(TLV_Buffer, AL_CALIB_AVG_TYPE, S.alCalibAvg);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    generateTLV_I(TLV_Buffer, AL_CALIB_COUNT_TYPE, 100);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    eof_size = addEOF(initLength + tlvSize);
    correctLengthInFrame(initLength, tlvSize, eof_size);
    return initLength + tlvSize + eof_size;
}

/* ------- (3) Hook BT_MC_parse_ALSettings into BT parse loop ----
 *
 * In the BT receive handler where BT.information is dispatched
 * (typically inside ParseBTMsg() or the BT state machine),
 * add:
 *
 *   case AL_SETTINGS_FROM_APP:           // 0x11
 *       if (!BT_MC_parse_ALSettings()) {
 *           // Send NACK / error back to app if needed
 *           HAL_UART_Transmit_IT(&huart2, (uint8_t *)SAVINGFAILURE, 6);
 *       } else {
 *           HAL_UART_Transmit_IT(&huart2, (uint8_t *)SAVINGSUCCESS, 6);
 *       }
 *       break;
 * ---------------------------------------------------------------- */


/* ------- (4) ACK error handling (add to MachineErrors.h) -------
 *
 * In MachineErrors.h, add a new error code:
 *   #define ERR_AL_SETTINGS_NO_ACK   <next available error code>
 *
 * This is triggered by FDCAN_ALSettings_CheckACK() after max retries.
 * ---------------------------------------------------------------- */



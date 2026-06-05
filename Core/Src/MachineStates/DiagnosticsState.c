/*
 * DiagnosticsState.c
 *
 *  Created on: Mar 7, 2023
 *      Author: harsha
 */

#include "Struct.h"
#include "MachineErrors.h"
#include "StateMachine.h"
#include "CommonConstants.h"
#include "FDCAN.h"
#include "CAN_MotherBoard.h"
#include "BT_Machine.h"
#include "userButtons.h"
#include "SysObserver.h"
#include "Log.h"

extern TIM_HandleTypeDef htim15;
extern UART_HandleTypeDef huart1;

void DiagnosticsState(void)
{
    uint8_t motorID    = 0;
    uint8_t noOfMotors = 0;
    uint8_t response   = 0;
    uint8_t motors[2]  = {0, 0};

    while (1) {

        if (usrBtns.redBtn == BTN_PRESSED) {
            usrBtns.redBtn = BTN_IDLE;
            S.oneTime  = 1;
            D.stopTest = 1;
        }

        if (S.oneTime == 1) {

            if (D.stopTest == 0) {
                /* ---- START command ---- */
                if (D.motorID <= 4) {
                    motorID    = GetMotorID_from_BTMotor_ID(D.motorID);
                    motors[0]  = motorID;
                    noOfMotors = 1;
                    response   = Send_DiagCommands_To_MultipleMotors(motors, noOfMotors, START);
                    if (response != 2) {
                        SO_enableCANObservers(&SO, motors, noOfMotors);
                        Log_setUpLogging(&L, motors, noOfMotors);
                    }
                } else if (D.motorID == BT_DRAFTING) {
                    motors[0]  = FR;
                    motors[1]  = BR;
                    noOfMotors = 2;
                    response   = Send_DiagCommands_To_MultipleMotors(motors, noOfMotors, START);
                    if (response != 2) {
                        SO_enableCANObservers(&SO, motors, noOfMotors);
                        Log_setUpLogging(&L, motors, noOfMotors);
                    }
                }

                /* Start TIM15 — sends runtime data to app */
                htim15.Instance->SR &= ~TIM_SR_UIF;
                HAL_TIM_Base_Start_IT(&htim15);

                /* *** ADD — if calibration mode, tell motor to start collecting */
                if (S.alCalibActive) {
                    FDCAN_SendCalibCommand_ToMotor(BACKROLLER_ADDRESS, 1);
                }

                S.oneTime  = 0;
                S.BT_dataOK = 0;

            } else {
                /* ---- STOP command ---- */

                /* *** ADD — if calibration was active, stop it on motor side */
                if (S.alCalibActive) {
                    FDCAN_SendCalibCommand_ToMotor(BACKROLLER_ADDRESS, 0);
                    S.alCalibActive      = 0;
                    S.alCalibResultReady = 0;
                }

                HAL_TIM_Base_Stop_IT(&htim15);

                uint8_t motorStop_Array[noOfMotors];
                for (int i = 0; i < noOfMotors; i++) {
                    motorStop_Array[i] = motors[i];
                }
                Send_DiagCommands_To_MultipleMotors(motorStop_Array, noOfMotors, EMERGENCY_STOP);
                SO_disableAndResetCANObservers(&SO);
                Log_disableLogging(&L);

                D.stopTest    = 0;
                S.oneTime     = 0;
                S.switchState = 0;
                ChangeState(&S, IDLE_STATE);
                break;
            }
        } /* closes oneTime */

        /* *** ADD — calibration result received from motor */
        if (S.alCalibActive && S.alCalibResultReady) {
            S.alCalibResultReady = 0;
            S.alCalibActive      = 0;

            /* Send average result to app */
            uint8_t packetSize = BT_MC_generateCalibResultMsg();
            while (S.BT_transmission_over != 1) {};
            HAL_UART_Transmit_IT(&huart1, (uint8_t *)BufferTransmit, packetSize);
            S.BT_transmission_over = 0;

            /* Auto stop motor after result sent */
            D.stopTest = 1;
            S.oneTime  = 1;
        }

        /* Error State */
        if (ME.ErrorFlag == 1) {
            ChangeState(&S, ERROR_STATE);
            break;
        }

        if (S.LOG_enabled) {
            Log_DoOneCycle();
        }

    } /* closes while */
}

/*
 * IdleState.c
 *  Created on: 15-Apr-2023
 *      Author: harsha
 */

#include "stdio.h"
#include "MachineErrors.h"
#include "StateMachine.h"
#include "CommonConstants.h"
#include "CAN_MotherBoard.h"
#include "MotorComms.h"
#include "Ack.h"
#include "userButtons.h"
#include "BT_Fns.h"
#include "TowerLamp.h"
#include "mcp23017.h"
#include "Log.h"
#include "BT_Machine.h"
#include "machineEepromSettings.h"
#include "Eeprom.h"

extern UART_HandleTypeDef huart1;

static float     last_kp      = 0.0f;
static uint16_t  last_sliver6 = 0;
static uint16_t  last_sliver5 = 0;
static uint16_t  last_sliver4 = 0;
static float     last_target  = 0.0f;

void IdleState(void)
{
    uint8_t response     = 0;
    uint8_t noOfMotors   = 0;
    uint8_t BTpacketSize = 0;

    while (1) {

        if (S.oneTime) {
            TowerLamp_SetState(&hmcp, &mcp_portB, BUZZER_OFF, RED_OFF, GREEN_OFF, AMBER_ON);
            TowerLamp_ApplyState(&hmcp, &mcp_portB);

            CalculateMachineParameters(&msp, &mcParams);
            Reset_CurrentMtrsRun(&mcParams);
            ReadySetupCommand_AllMotors(&msp, &mcParams);
            FDCAN_SendMachineCalculationTOBR_AL();
            mcParams.currentMtrsRun = 0.01;

            L.logRunStateChange = 1;
            L.flushBuffer = 1;

            S.alSettingsSent      = 0;
            ALSendState.ackStatus = AL_SETTINGS_ACK_PENDING;
            ALSendState.retryCnt  = 0;

            /* Read EEPROM once — send saved AL values to motor */
            uint16_t alValidFlag = EE_ReadInteger(AL_VALID_FLAG_ADDR);
            if (alValidFlag == AL_VALID_FLAG_VALUE) {
                last_kp      = al_bt.Kp;
                last_sliver6 = al_bt.sliver_nplus1;
                last_sliver5 = al_bt.sliver_n;
                last_sliver4 = al_bt.sliver_nminus1;
                last_target  = al_bt.target_g_per_m;

                if (S.alSettingsJustSaved) {
                    /* SettingsState already sent — skip resend, just update last_* */
                    S.alSettingsJustSaved = 0;
                } else {
                    FDCAN_SendALSettings_ToMotor(BACKROLLER_ADDRESS,
                        last_kp, last_sliver6, last_sliver5,
                        last_sliver4, last_target);
                }
                S.alSettingsSent = 1;
            }

            S.oneTime = 0;
        }

        /* Retry check — self-throttled by timestamp */
        if (S.alSettingsSent && ALSendState.ackStatus == AL_SETTINGS_ACK_PENDING) {
            FDCAN_ALSettings_CheckACK(BACKROLLER_ADDRESS,
                last_kp, last_sliver6, last_sliver5,
                last_sliver4, last_target);
        }

        /* If all retries failed — will retry on next idle entry */
        if (ALSendState.ackStatus == AL_SETTINGS_ACK_FAIL) {
            S.alSettingsSent      = 0;
            ALSendState.ackStatus = AL_SETTINGS_ACK_PENDING;
            ALSendState.retryCnt  = 0;
        }

        /* Yellow button — inching */
        if ((usrBtns.yellowBtn == BTN_PRESSED) && (usrBtns.logicApplied == 0)) {
            CalculateMachineParameters(&ps, &mcParams);
            ReadySetupCommand_AllMotors(&ps, &mcParams);
            usrBtns.logicApplied = 1;
            uint8_t motors[] = {FR, BR, CREEL};
            noOfMotors = 3;
            response = SendCommands_To_MultipleMotors(motors, noOfMotors, START);
        }

        if ((usrBtns.yellowBtn == BTN_IDLE) && (usrBtns.logicApplied == 1)) {
            usrBtns.logicApplied = 0;
            uint8_t motors[] = {FR, BR, CREEL};
            noOfMotors = 3;
            response = SendCommands_To_MultipleMotors(motors, noOfMotors, EMERGENCY_STOP);
        }

        /* Green button — start run */
        if (usrBtns.greenBtn == BTN_PRESSED) {
            usrBtns.greenBtn = BTN_IDLE;
            if (ALSendState.ackStatus == AL_SETTINGS_ACK_OK ||
                ALSendState.ackStatus == AL_SETTINGS_ACK_PENDING) {
                Log_ResetRunTimeRdngNos();
                ChangeState(&S, RUN_STATE);
                break;
            } else {
                /* CAN failure — block start, alert operator */
                TowerLamp_SetState(&hmcp, &mcp_portB, BUZZER_ON, RED_ON, GREEN_OFF, AMBER_OFF);
                TowerLamp_ApplyState(&hmcp, &mcp_portB);
            }
        }

        if (S.switchState == TO_SETTINGS) {
            ChangeState(&S, SETTINGS_STATE);
            S.switchState = 0;
            break;
        } else if (S.switchState == TO_DIAGNOSTICS) {
            ChangeState(&S, DIAGNOSTICS_STATE);
            S.switchState = 0;
            break;
        }

        if (ME.ErrorFlag == 1) {
            ChangeState(&S, ERROR_STATE);
            break;
        }

        /* BT status send — 500ms timer */
        if ((S.BT_sendState == 1) && (S.BT_transmission_over == 1)) {
            BTpacketSize = BT_MC_generateStatusMsg(BT_IDLE);
            HAL_UART_Transmit_IT(&huart1, (uint8_t *)BufferTransmit, BTpacketSize);
            S.BT_transmission_over = 0;
            S.BT_sendState = 0;
        }

        if (S.LOG_enabled) Log_DoOneCycle();

        if (S.current_state != IDLE_STATE) break;

    }
}

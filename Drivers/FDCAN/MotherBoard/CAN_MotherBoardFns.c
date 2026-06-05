/*
 * CAN_MotherBoardFns.c
 *  Created on: Mar 7, 2023
 *      Author: harsha
 */
#include "main.h"
#include "stm32g4xx_hal.h"
#include "CAN_MotherBoard.h"
#include "StateMachine.h"

ALSettingsSendState_TypeDef ALSendState = {0};
static uint8_t lastSensorToggle = 0xFF;

#define AL_SETTINGS_MAX_RETRIES  3

/*
 * FDCAN_SendALSettings_ToMotor()
 * Sends KP, Sliver6/5/4, TARGET_G to BR motor.
 * Frame ID: 0x0A240301 (functionID=0x24, dst=0x03, src=0x01)
 * Data: [0-1] KP×1000, [2-3] S6, [4-5] S5, [6-7] S4, [8-9] TARGET×100
 */
void FDCAN_SendALSettings_ToMotor(uint8_t destination,
                                  float kp,
                                  uint16_t sliver6,
                                  uint16_t sliver5,
                                  uint16_t sliver4,
                                  float target_g_per_m)
{
    uint16_t kp_out   = (uint16_t)(kp * 1000.0f);
    uint16_t tgpm_out = (uint16_t)(target_g_per_m * 100.0f);

    TxHeader.Identifier = (0x0A24 << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_12;
    TxData[0]  = kp_out >> 8;     TxData[1]  = kp_out & 0xFF;
    TxData[2]  = sliver6 >> 8;    TxData[3]  = sliver6 & 0xFF;
    TxData[4]  = sliver5 >> 8;    TxData[5]  = sliver5 & 0xFF;
    TxData[6]  = sliver4 >> 8;    TxData[7]  = sliver4 & 0xFF;
    TxData[8]  = tgpm_out >> 8;   TxData[9]  = tgpm_out & 0xFF;
    TxData[10] = 0;                TxData[11] = 0;

    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

    ALSendState.ackStatus     = AL_SETTINGS_ACK_PENDING;
    ALSendState.sentTimestamp = HAL_GetTick();
}

/*
 * FDCAN_Receive_ALSettingsACK()
 * Called when mainboard receives ACK (functionID 0x24) from motor.
 */
void FDCAN_Receive_ALSettingsACK(void)
{
    if (RxData[0] == 1) {
        ALSendState.ackStatus = AL_SETTINGS_ACK_OK;
        ALSendState.retryCnt  = 0;
    } else {
        ALSendState.ackStatus = AL_SETTINGS_ACK_FAIL;
    }
}

/*
 * FDCAN_ALSettings_CheckACK()
 * Call from idle loop — retries up to AL_SETTINGS_MAX_RETRIES on timeout.
 */
void FDCAN_ALSettings_CheckACK(uint8_t destination,
                                float kp, uint16_t sliver6,
                                uint16_t sliver5, uint16_t sliver4,
                                float target_g_per_m)
{
    if (ALSendState.ackStatus != AL_SETTINGS_ACK_PENDING) return;

    uint32_t elapsed = HAL_GetTick() - ALSendState.sentTimestamp;
    if (elapsed >= AL_SETTINGS_ACK_TIMEOUT_MS) {
        if (ALSendState.retryCnt < AL_SETTINGS_MAX_RETRIES) {
            ALSendState.retryCnt++;
            FDCAN_SendALSettings_ToMotor(destination, kp,
                                         sliver6, sliver5, sliver4,
                                         target_g_per_m);
        } else {
            ALSendState.ackStatus = AL_SETTINGS_ACK_FAIL;
            ME_addErrors(&ME, ERR_MOTOR_SOURCE, ERR_AL_SETTINGS_NO_ACK, 0, 0);
        }
    }
}




/* ---- Standard motor communication functions ---- */

void FDCAN_sendCommand_ToMotor(uint8_t destination, uint8_t command)
{
    TxHeader.Identifier = (0x601 << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxData[0] = command;
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_SendMachineCalculationTOBR_AL(void)
{
    TxHeader.Identifier = (0xA1F03 << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_5;   // *** was BYTES_4

    TxData[0] = ((uint16_t)(msp.draft * 100)) >> 8;
    TxData[1] = (uint16_t)(msp.draft * 100);
    TxData[2] = msp.delivery_mMin >> 8;
    TxData[3] = msp.delivery_mMin;
    TxData[4] = S.sensorToggleState;   // virtual toggle from app

    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_sendSetUp_ToMotor(uint8_t destination, SetupMotor rd)
{
    TxHeader.Identifier = (0xA07 << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxData[0] = rd.RUT;
    TxData[1] = rd.RDT;
    TxData[2] = rd.RPM >> 8;
    TxData[3] = rd.RPM;
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_SendDiagnostics_ToMotor(uint8_t destination, DiagnosticsTypeDef *d)
{
    TxHeader.Identifier = (0xE0A << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxData[0] = d->typeofTest;
    if (d->typeofTest == OPENLOOP) {
        TxData[1] = d->targetDuty >> 8;
        TxData[2] = d->targetDuty;
    } else {
        TxData[1] = (d->targetRPM >> 8) & 0xFF;
        TxData[2] = d->targetRPM & 0xFF;
    }
    TxData[3] = d->rampUpTime;
    TxData[4] = d->rampDownTime;
    TxData[5] = d->runTime >> 8;
    TxData[6] = d->runTime;
    TxData[7] = d->direction;
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_sendConsoleCHK_Response(uint8_t destination)
{
    TxHeader.Identifier = (0xE19 << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxData[0] = 1;
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_sendChangeTarget_ToMotor(uint8_t destination, uint16_t newTarget, uint16_t transitionTime_ms)
{
    TxHeader.Identifier = (0xA0D << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxData[0] = newTarget >> 8;       TxData[1] = newTarget;
    TxData[2] = transitionTime_ms >> 8; TxData[3] = transitionTime_ms;
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_sendDataRequest_ToMotor(uint8_t destination, uint8_t requestType)
{
    TxHeader.Identifier = (0xA06 << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxData[0] = requestType;
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_Recieve_ACKFromMotors(uint8_t sourceAddress)
{
    ack.motorAcksRecvd |= (1 << (sourceAddress - 2));
    if (ack.motorAcksCheck == ack.motorAcksRecvd) {
        ack.waitingForAckResult = 0;
        ack.ackResult = ACK_SUCCESS;
        HAL_TIM_Base_Stop_IT(&htim17);
    }
}

void FDCAN_Recieve_DataResponse(DataReq *d, uint8_t motorID)
{
    d->p.motorID             = RxData[0];
    d->p.Kp_motorID          = (RxData[1] << 8) | RxData[2];
    d->p.Ki_motorID          = (RxData[3] << 8) | RxData[4];
    d->p.FF_motorID          = (RxData[5] << 8) | RxData[6];
    d->p.startOffset_motorID = (RxData[7] << 8) | RxData[8];
}

void FDCAN_sendPID_Update_toMotor(uint8_t destination)
{
    TxHeader.Identifier = (0xE1A << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_12;
    TxData[0] = DR.requestType;
    uint16_t temp = DR.p.Kp_motorID;
    TxData[1] = temp >> 8; TxData[2] = temp;
    temp = DR.p.Ki_motorID;
    TxData[3] = temp >> 8; TxData[4] = temp;
    temp = DR.p.FF_motorID;
    TxData[5] = temp >> 8; TxData[6] = temp;
    temp = DR.p.startOffset_motorID;
    TxData[7] = temp >> 8; TxData[8] = temp;
    TxData[9] = 0; TxData[10] = 0; TxData[11] = 0;
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_Recieve_RunDataFromMotors(uint8_t motorID)
{
    if (motorID <= 6) {
        R[motorID].rdngNo++;
        R[motorID].targetRPM  = (RxData[0] << 8) | RxData[1];
        R[motorID].presentRPM = (RxData[2] << 8) | RxData[3];
        R[motorID].pwm        = (RxData[4] << 8) | RxData[5];
        if (functionID == RUNTIMEDATA_FUNCTIONID) {
            R[motorID].mosfetTemp = RxData[6];
            R[motorID].motorTemp  = RxData[7];
            R[motorID].currentRaw = (RxData[8] << 8)  | RxData[9];
            R[motorID].voltageRaw = (RxData[10] << 8) | RxData[11];
            R[motorID].currentA   = (float)R[motorID].currentRaw * MOTORBRD_CURRENT_GAIN;
            R[motorID].voltageV   = (float)R[motorID].voltageRaw * MOTORBRD_VOLTAGE_GAIN;
            R[motorID].power      = R[motorID].currentA * R[motorID].voltageV;
        }
        if (functionID == ANALYSISDATA_FUNCTIONID) {
            R[motorID].proportionalTerm = (RxData[6] << 8)  | RxData[7];
            R[motorID].integralTerm     = (RxData[8] << 8)  | RxData[9];
            R[motorID].feedForwardTerm  = (RxData[10] << 8) | RxData[11];
        }
    }
}

void FDCAN_Recieve_ErrorsFromMotors(uint8_t sourceAddress)
{
    uint8_t motorID = GetMotorID_from_CANAddress(sourceAddress);
    uint16_t temp = (RxData[0] << 8) | RxData[1];
    R[motorID].motorError = temp;
    ME.ErrorFlag = 1;
    uint16_t motorErrReason = FindTopMotorError(&ME, temp);
    ME_addErrors(&ME, ERR_MOTOR_SOURCE, motorErrReason, motorID, temp);
}

void FDCAN_ACK_BR(void)
{
    S.AL_ack_received = RxData[0];
}
// Send start/stop calibration command to motor
void FDCAN_SendCalibCommand_ToMotor(uint8_t destination, uint8_t command)
{
    TxHeader.Identifier = (0x0A27 << 16) | (destination << 8) | 0x01;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxData[0] = command;
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

// Receive average result from motor
void FDCAN_Receive_CalibResult(void)
{
    S.alCalibAvg         = (RxData[0] << 8) | RxData[1];
    S.alCalibResultReady = 1;
}
void FDCAN_parseForMotherBoard(void)
{
    uint8_t motorID;
    functionID     = (RxHeader.Identifier & 0xFF0000) >> 16;
    source_address = RxHeader.Identifier & 0xFF;

    switch (functionID) {
        case ERROR_FUNCTIONID:
            FDCAN_Recieve_ErrorsFromMotors(source_address);
            break;
        case ACKFRAME_FUNCTIONID:
            FDCAN_Recieve_ACKFromMotors(source_address);
            break;
        case RUNTIMEDATA_FUNCTIONID:
            motorID = GetMotorID_from_CANAddress(source_address);
            FDCAN_Recieve_RunDataFromMotors(motorID);
            SO_incrementCANCounter(&SO, motorID);
            break;
        case DIAGNOSTICSDONEFRAME_FUNCTIONID:
            SO_disableAndResetCANObservers(&SO);
            break;
        case DRIVE_CAN_CHK_REQUEST:
            FDCAN_sendConsoleCHK_Response(source_address);
            break;
        case DATA_REQUEST_RESPONSE:
            if (DR.requestSent == 1 && DR.requestType == PID_SETTINGS_REQUEST) {
                motorID = GetMotorID_from_CANAddress(source_address);
                FDCAN_Recieve_DataResponse(&DR, motorID);
                DR.responseRecieved = 1;
            }
            break;
        case ACK_FROM_BR_CALCULATIONS:
            FDCAN_ACK_BR();
            if (S.AL_ack_received == 0)
                FDCAN_SendMachineCalculationTOBR_AL();
            break;
        case PID_UPDATE_RESPONSE:
            if (DR.requestSent == 1) {
                DR.responseSuccess  = (RxData[0] == 1) ? 1 : 0;
                DR.responseRecieved = 1;
            }
            break;
        case AL_SETTINGS_FUNCTIONID:
            FDCAN_Receive_ALSettingsACK();
            break;

        case AL_CALIB_RESULT_FUNCTIONID:   // 0x28
            FDCAN_Receive_CalibResult();
            break;
    }
}

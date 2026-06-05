/*
 * BT_Fns.c
 *
 *  Created on: Mar 8, 2023
 *      Author: harsha
 */

#include "BT_Fns.h"
#include "BT_Machine.h"
#include "machineSettings.h"
#include "MachineErrors.h"
#include "StateMachine.h"


char paired[8];
char sof[2];
char msgLength[2];
char eof[2];
char information[2];
char subState[2];
char attributeCount[2];

char charToHexDigit(char c)
{
    if (islower(c)) c = toupper(c);
    if (c >= 'A') return c - 'A' + 10;
    else          return c - '0';
}

char StringToHex(char c[2])
{
    return charToHexDigit(c[1]) + 16 * charToHexDigit(c[0]);
}

uint16_t stringToINT(char c[4])
{
    return charToHexDigit(c[3]) + 16 * charToHexDigit(c[2])
         + 256 * charToHexDigit(c[1]) + 4096 * charToHexDigit(c[0]);
}

uint8_t DecodeStringAsUint8(char c[2])
{
    return charToHexDigit(c[1]) + charToHexDigit(c[0]) * 10;
}

uint16_t DecodeStringAsInteger(char c[4])
{
    return charToHexDigit(c[3]) + charToHexDigit(c[2]) * 10
         + charToHexDigit(c[1]) * 100 + charToHexDigit(c[0]) * 1000;
}

unsigned long stringToFLOAT(char c[8])
{
    return charToHexDigit(c[7])
         + 16UL        * charToHexDigit(c[6])
         + 256UL       * charToHexDigit(c[5])
         + 4096UL      * charToHexDigit(c[4])
         + 65536UL     * charToHexDigit(c[3])
         + 1048576UL   * charToHexDigit(c[2])
         + 16777216UL  * charToHexDigit(c[1])
         + 268435456UL * charToHexDigit(c[0]);
}

void ParseBTMsg(void)
{
    strncpy(sof,           BufferRec,              2);
    strncpy(msgLength,     BufferRec + 2,          2);
    BT.sof    = StringToHex(sof);
    BT.length = StringToHex(msgLength);
    strncpy(eof,           BufferRec + 4 + BT.length - 2, 2);
    BT.eof = StringToHex(eof);
    strncpy(information,   BufferRec + 4,          2);
    strncpy(subState,      BufferRec + 6,          2);
    strncpy(attributeCount,BufferRec + 8,          2);
    BT.information  = StringToHex(information);
    BT.subState     = StringToHex(subState);
    BT.attributeCnt = DecodeStringAsUint8(attributeCount);
    BT.TLVsLength   = BT.length - 8;

    BT.BTmsgGood = ((BT.sof == 0x7E) & (BT.eof == 0x7E)) ? 1 : 0;
}

void generateTLV_C(char *buffer, uint8_t ID, uint16_t value)
{
    sprintf(buffer, "%02X%02X%02X", ID, 2, value);
}

void generateTLV_I(char *buffer, uint8_t ID, uint16_t value)
{
    sprintf(buffer, "%02X%02X%04X", ID, 4, value);
}

void generateTLV_F(char *buffer, uint8_t ID, float valueF)
{
    sprintf(buffer, "%02X%02X%X", ID, 8, *(unsigned int *)&valueF);
}

void add_TLVBuf_To_TxBuf(char *TLVbuf, uint8_t copySize, uint8_t TXBuf_location)
{
    for (int i = 0; i < copySize; i++)
        BufferTransmit[TXBuf_location + i] = TLVbuf[i];
}

uint8_t Init_TXBuf_Frame(uint8_t information, uint8_t substate, uint8_t TLV_count)
{
    sprintf(BufferTransmit, "%02X%02X%02X%02X%02d",
            FRAME_DELIMITER, 0, information, substate, TLV_count);
    return 10;
}

uint8_t addEOF(uint8_t location)
{
    BufferTransmit[location]     = '7';
    BufferTransmit[location + 1] = 'E';
    return 2;
}

void correctLengthInFrame(uint8_t initLength, uint8_t TLV_length, uint8_t eof_length)
{
    uint8_t framelength = TLV_length + eof_length + (initLength - 4);
    char buffer[3];
    sprintf(buffer, "%02X", framelength);
    BufferTransmit[2] = buffer[0];
    BufferTransmit[3] = buffer[1];
}

uint8_t BT_MC_generateALSettingsMsg(void)
{
    char TLV_Buffer[12];
    uint8_t tlvSize   = 0;
    uint8_t eof_size  = 0;
    uint8_t initLength = Init_TXBuf_Frame(AL_SETTINGS_FROM_MB, SUBSTATE_NA, 5);

    generateTLV_F(TLV_Buffer, AL_KP_TYPE, al_bt.Kp);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
    tlvSize += TLV_FLOAT;

    generateTLV_I(TLV_Buffer, AL_SLIVER6_TYPE, al_bt.sliver_nplus1);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    generateTLV_I(TLV_Buffer, AL_SLIVER5_TYPE, al_bt.sliver_n);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    generateTLV_I(TLV_Buffer, AL_SLIVER4_TYPE, al_bt.sliver_nminus1);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    generateTLV_F(TLV_Buffer, AL_TARGET_G_TYPE, al_bt.target_g_per_m);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
    tlvSize += TLV_FLOAT;

    eof_size = addEOF(initLength + tlvSize);
    correctLengthInFrame(initLength, tlvSize, eof_size);
    return initLength + tlvSize + eof_size;
}

uint8_t parseTLV(TLVStruct_TypeDef *T, uint8_t startLocation)
{
    char type[2], len[2], V_C[2], V_I[4], V_F[8];
    unsigned long float_hex;
    uint8_t TLV_size = 0;

    strncpy(type, BufferRec + startLocation,     2);
    strncpy(len,  BufferRec + startLocation + 2, 2);
    T->type   = StringToHex(type);
    T->length = StringToHex(len);

    if (T->length == 2) {
        strncpy(V_C, BufferRec + startLocation + 4, 2);
        T->value_int = StringToHex(V_C);
        TLV_size = 6;
    }
    if (T->length == 4) {
        strncpy(V_I, BufferRec + startLocation + 4, 4);
        T->value_int = stringToINT(V_I);
        TLV_size = 8;
    } else if (T->length == 8) {
        strncpy(V_F, BufferRec + startLocation + 4, 8);
        float_hex  = stringToFLOAT(V_F);
        T->value_f = *((float *)&float_hex);
        TLV_size = 12;
    }
    return TLV_size;
}

uint8_t ParseDiagnostics(void)
{
    TLVStruct_TypeDef T;
    uint8_t TLV_start = 10, tlvSize = 0, count = 0;
    uint8_t allSettingsRecieved = 0, rampTime = 0;

    for (int i = 0; i < BT.attributeCnt; i++) {
        tlvSize = parseTLV(&T, TLV_start);
        TLV_start += tlvSize;
        switch (T.type) {
            case MOTOR_ID:
                D.motorID = T.value_int;
                count++;
                break;
            case CONTROL_TYPE:
                D.typeofTest = T.value_int;
                count++;
                break;
            case TARGET_PERCENT:
                if (D.typeofTest == OPENLOOP) {
                    uint16_t temp = (uint16_t)(T.value_int * 1499 / 100);
                    D.targetDuty  = temp + (10 - (temp % 10));
                    rampTime      = (D.targetDuty < 800) ? 5 : 10;
                    count++;
                }
                if (D.typeofTest == CLOSEDLOOP) {
                    D.targetRPM = (uint16_t)(T.value_int);
                    rampTime    = (D.targetRPM < 800) ? 5 : 10;
                    count++;
                }
                D.rampUpTime   = rampTime;
                D.rampDownTime = rampTime;
                break;
            case TARGET_RUNTIME:
                D.runTime = T.value_int;
                count++;
                break;
            case MOTOR_DIR:
                D.direction = T.value_int;
                count++;
                break;
        }
    }

    if (D.motorID < 6 && count == 5)
        allSettingsRecieved = 1;

    return allSettingsRecieved;
}

void InitializeBTMsg_TypeDef(BTRecvMsg_TypeDef *b)
{
    b->sof = 0; b->eof = 0; b->length = 0;
    b->attributeCnt = 0; b->information = 0;
    b->subState = 0; b->TLVsLength = 0; b->BTmsgGood = 0;
}

uint8_t BT_MC_generateDiagnosticMsg(uint8_t BT_MotorID)
{
    char TLV_Buffer[12];
    uint8_t tlvSize = 0, eof_size = 0, initLength = 0;
    uint8_t motorID = GetMotorID_from_BTMotor_ID(BT_MotorID);
    uint8_t noOfAttributes = (motorID < 5) ? 4 : 5;

    initLength = Init_TXBuf_Frame(DIAG_RUN_DATA_TO_APP, SUBSTATE_NA, noOfAttributes);

    generateTLV_I(TLV_Buffer, DIAG_SPEED_DATA, R[motorID].presentRPM);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    generateTLV_I(TLV_Buffer, DIAG_PWM_DATA, R[motorID].pwm);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    if (R[motorID].currentA == 0) R[motorID].currentA = 0.001f;
    generateTLV_F(TLV_Buffer, DIAG_CURRENT_DATA, R[motorID].currentA);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
    tlvSize += TLV_FLOAT;

    if (R[motorID].power == 0) R[motorID].power = 0.001f;
    generateTLV_F(TLV_Buffer, DIAG_POWER_DATA, R[motorID].power);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
    tlvSize += TLV_FLOAT;

    eof_size = addEOF(initLength + tlvSize);
    correctLengthInFrame(initLength, tlvSize, eof_size);
    return initLength + tlvSize + eof_size;
}

uint8_t BT_MC_generateDiagnosticMsg_DoubleMotors(uint8_t diagCommand)
{
    char TLV_Buffer[12];
    uint8_t tlvSize = 0, eof_size = 0, initLength = 0;
    uint8_t firstMotor = 0, secondMotor = 0, noOfAttributes = 0;

    if (diagCommand == BT_DRAFTING) {
        firstMotor     = FR;
        secondMotor    = BR;
        noOfAttributes = 8;
    }

    initLength = Init_TXBuf_Frame(DIAG_RUN_DATA_TO_APP, SUBSTATE_NA, noOfAttributes);

    generateTLV_I(TLV_Buffer, DIAG_SPEED_DATA, R[firstMotor].presentRPM);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    generateTLV_I(TLV_Buffer, DIAG_PWM_DATA, R[firstMotor].pwm);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    if (R[firstMotor].currentA == 0) R[firstMotor].currentA = 0.001f;
    generateTLV_F(TLV_Buffer, DIAG_CURRENT_DATA, R[firstMotor].currentA);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
    tlvSize += TLV_FLOAT;

    if (R[firstMotor].power == 0) R[firstMotor].power = 0.001f;
    generateTLV_F(TLV_Buffer, DIAG_POWER_DATA, R[firstMotor].power);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
    tlvSize += TLV_FLOAT;

    generateTLV_I(TLV_Buffer, DIAG_SPEED_DATA, R[secondMotor].presentRPM);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    generateTLV_I(TLV_Buffer, DIAG_PWM_DATA, R[secondMotor].pwm);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
    tlvSize += TLV_INT;

    if (R[secondMotor].currentA == 0) R[secondMotor].currentA = 0.001f;
    generateTLV_F(TLV_Buffer, DIAG_CURRENT_DATA, R[secondMotor].currentA);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
    tlvSize += TLV_FLOAT;

    if (R[secondMotor].power == 0) R[secondMotor].power = 0.001f;
    generateTLV_F(TLV_Buffer, DIAG_POWER_DATA, R[secondMotor].power);
    add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
    tlvSize += TLV_FLOAT;

    eof_size = addEOF(initLength + tlvSize);
    correctLengthInFrame(initLength, tlvSize, eof_size);
    return initLength + tlvSize + eof_size;
}

uint8_t BT_MC_generateStatusMsg(uint8_t state)
{
    char TLV_Buffer[12];
    uint8_t tlvSize = 0, eof_size = 0, initLength = 0;
    uint8_t runAttributes = 0;

    if ((state == BT_INCHING) || (state == BT_IDLE) || (state == BT_FINISH)) {
        /* 2 attributes: placeholder + toggle state */
        initLength = Init_TXBuf_Frame(MC_STATE, state, 2);

        generateTLV_I(TLV_Buffer, 0x00, 0);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
        tlvSize += TLV_INT;

        /* Toggle always sent so app knows current AL state in idle */
        generateTLV_I(TLV_Buffer, RUN_SENSOR_TOGGLE_STATE, S.sensorToggleState);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
        tlvSize += TLV_INT;

    } else if (state == BT_PAUSE) {
        initLength = Init_TXBuf_Frame(MC_STATE, state, 2);

        generateTLV_I(TLV_Buffer, BT_PAUSE_REASON, S.BT_pauseReason);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
        tlvSize += TLV_INT;

        generateTLV_F(TLV_Buffer, PAUSE_DF_OUTPUT_LENGTH, mcParams.currentMtrsRun);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
        tlvSize += TLV_FLOAT;

    } else if (state == BT_RUN) {
        /* Toggle always added — 1 extra attribute in both views */
        if (S.BT_runInfoToSend == RUN_PRODUCTION_DATA) {
            runAttributes = 5;   /* 4 production TLVs + 1 toggle */
        } else {
            runAttributes = 7;   /* 6 motor data TLVs + 1 toggle */
        }
        initLength = Init_TXBuf_Frame(MC_STATE, state, runAttributes);

        generateTLV_C(TLV_Buffer, RUN_TLV_WHAT_INFO, S.BT_runInfoToSend);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLC_C, initLength + tlvSize);
        tlvSize += TLC_C;

        generateTLV_F(TLV_Buffer, RUN_DF_OUTPUT_LENGTH, mcParams.currentMtrsRun);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
        tlvSize += TLV_FLOAT;

        if (S.BT_runInfoToSend == RUN_PRODUCTION_DATA) {
            generateTLV_F(TLV_Buffer, RUN_OUTPUT_MTRS_PER_SPINDLE, 342.11f);
            add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
            tlvSize += TLV_FLOAT;

            generateTLV_F(TLV_Buffer, RUN_TOTAL_POWER, mcParams.totalPower);
            add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
            tlvSize += TLV_FLOAT;

        } else {
            generateTLV_I(TLV_Buffer, RUN_TLV_MOTOR_TEMP, R[S.BT_runInfowhichMotor].motorTemp);
            add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
            tlvSize += TLV_INT;

            generateTLV_I(TLV_Buffer, RUN_TLV_MOSFET_TEMP, R[S.BT_runInfowhichMotor].mosfetTemp);
            add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
            tlvSize += TLV_INT;

            generateTLV_F(TLV_Buffer, RUN_TLV_MOTOR_CURRENT, R[S.BT_runInfowhichMotor].currentA);
            add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_FLOAT, initLength + tlvSize);
            tlvSize += TLV_FLOAT;

            generateTLV_I(TLV_Buffer, RUN_TLV_MOTOR_RPM, R[S.BT_runInfowhichMotor].presentRPM);
            add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
            tlvSize += TLV_INT;
        }

        /* Toggle ALWAYS outside if/else — sent in both production and motor views */
        generateTLV_I(TLV_Buffer, RUN_SENSOR_TOGGLE_STATE, S.sensorToggleState);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
        tlvSize += TLV_INT;
        S.sensorToggleUpdated = 0;

    } else if (state == BT_STOP) {
        initLength = Init_TXBuf_Frame(MC_STATE, state, 4);

        generateTLV_I(TLV_Buffer, ERR_REASON, ME.BT_errorReason);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
        tlvSize += TLV_INT;

        generateTLV_I(TLV_Buffer, ERR_SOURCE, ME.BT_errorSrc);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
        tlvSize += TLV_INT;

        generateTLV_I(TLV_Buffer, ERR_CODE, ME.BT_errorCode);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
        tlvSize += TLV_INT;

        generateTLV_I(TLV_Buffer, ERR_DF_OUTPUT_LENGTH, mcParams.currentMtrsRun);
        add_TLVBuf_To_TxBuf(TLV_Buffer, TLV_INT, initLength + tlvSize);
        tlvSize += TLV_INT;
    }

    eof_size = addEOF(initLength + tlvSize);
    correctLengthInFrame(initLength, tlvSize, eof_size);
    return initLength + tlvSize + eof_size;
}

uint8_t checkConnectionStatus(void)
{
    strncpy(paired, BufferRec + 1, 8);
    paired[7] = '\0';
    if (strcmp(paired, BT_CONNECT_MSG) == 0) {
        S.BT_Paired = 1;
        return CONNECTION_MADE;
    }
    if (strcmp(paired, BT_DISCONNECT_MSG) == 0) {
        S.BT_Paired = 0;
        return CONNECTION_LOST;
    }
    return 0;
}

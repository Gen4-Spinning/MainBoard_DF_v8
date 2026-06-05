/*
 * EepromSettings.h
 *
 *  Created on: 06-Mar-2023
 *      Author: harsha
 */

#ifndef INC_EEPROMSETTINGS_H_
#define INC_EEPROMSETTINGS_H_

//Addresses
//Dont let a address go across address 32 and its multiples. Thats one page.
#define DF_DELIVERY_M_MIN_ADDR 0X02 //int
#define DF_DRAFT_ADDR 0X06 // float
#define DF_LENGTH_LIMIT_M 0X0A //int
#define RAMPUP_TIME_ADDR 0X0C //int
#define RAMPDOWN_TIME_ADDR 0X0E // int
#define CREEL_TENSION_ADDR 0X12 // float

//DEFAULTS
#define DEFAULT_DELIVERY_M_MIN 80
#define DEFAULT_DRAFT 8
#define DEFAULT_LENGTHLIMIT 100
#define DEFAULT_RAMPUP_TIME 6		// in sec
#define DEFAULT_RAMPDOWN_TIME 6	// in sec
#define DEFAULT_CREELTENSION_FACTOR 1

// AL Settings addresses — starts at 0x16, ends at 0x1F (fits in page 0)
#define AL_KP_ADDR              0x16    // float  — 4 bytes (0x16-0x19)
#define AL_SLIVER_NPLUS1_ADDR         0x1A    // int    — 2 bytes (0x1A-0x1B)
#define AL_SLIVER_N_ADDR         0x1C    // int    — 2 bytes (0x1C-0x1D)
#define AL_SLIVER_NMINUS1_ADDR         0x1E    // int    — 2 bytes (0x1E-0x1F)
// page boundary at 0x20 — next page starts here
#define AL_TARGET_G_ADDR        0x20    // float  — 4 bytes (0x20-0x23)
#define AL_VALID_FLAG_ADDR      0x24    // int    — 2 bytes
#define AL_VALID_FLAG_VALUE     0xBF  // magic number to confirm valid data

// AL Defaults
#define DEFAULT_AL_KP           0.04f
#define DEFAULT_AL_SLIVER6      605
#define DEFAULT_AL_SLIVER5      780
#define DEFAULT_AL_SLIVER4      989
#define DEFAULT_AL_TARGET_G     5.0f

//defaults for piecing
#define PIECING_DELIVERY_M_MIN 20
#define PIECING_RAMPUP_TIME 2
#define PIECING_RAMPDOWN_TIME 2

void ReadALSettingsFromEeprom(ALSettings_BT_TypeDef *al);
uint8_t WriteALSettingsIntoEeprom(ALSettings_BT_TypeDef *al);
uint8_t CheckALSettings(ALSettings_BT_TypeDef *al);
void LoadDefaultALSettings(ALSettings_BT_TypeDef *al);
#endif /* INC_EEPROMSETTINGS_H_ */

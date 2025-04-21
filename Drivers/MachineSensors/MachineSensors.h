/*
 * MachineSensors.h
 *
 *  Created on: May 11, 2023
 *      Author: harsha
 */

#ifndef MACHINESENSORS_H_
#define MACHINESENSORS_H_

#include "mcp23017.h"

#define UNKNOWN_SENSOR 0
#define CREEL_SLIVER_CUT_SENSOR 1
#define LAPPING_SENSOR 2

#define SENSOR_ENABLE 1
#define SENSOR_DISABLE 2
typedef struct {
	int8_t lappingSensor;
	uint8_t lappingTimerIncrementBool;
	uint8_t lappingSensorTimer;
	uint8_t lappingSensorOneShot;

	uint8_t sliverCutSensor;
	uint8_t sliverCutTimerIncrementBool;
	uint8_t slivercutSensorTimer;
	uint8_t sliverCutOneShot;
} SensorTypeDef;

extern SensorTypeDef sensor;

uint8_t Sensor_whichTriggered(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *whichSensor);
void Sensor_resetTriggeredStates(MCP23017_PortB *whichSensor);
int8_t Sensor_GetTriggerValue(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor);
void LappingSensorMonitor(SensorTypeDef *s);
int8_t Sensor_ReadValueDirectly(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor);
void SliverSensorMonitor(SensorTypeDef *s);

#endif /* MACHINESENSORS_H_ */

/*
 * MachineSensors.c
 *
 *  Created on: May 11, 2023
 *      Author: harsha
 */


#include "MachineSensors.h"

uint8_t Sensor_whichTriggered(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *whichSensor){
	mcp23017_read(mcp, INTFB,mcp->intTrigger);// intFB gives which pin has interrupt
	whichSensor->raw = mcp->intTrigger[0];
	if (whichSensor->values.input0 == 1){
		return LAPPING_SENSOR;
	}
	if (whichSensor->values.input1 == 1){
		return CREEL_SLIVER_CUT_SENSOR;
	}
	return UNKNOWN_SENSOR;
}


void Sensor_resetTriggeredStates(MCP23017_PortB *whichSensor){
	whichSensor->raw = 0 ;
}

int8_t Sensor_GetTriggerValue(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor){
	mcp23017_read(mcp, INTCAPB,mcp->intTriggerCapturedValue); // captures GPIO value when interrupt comes.
	sensorVal->raw = mcp->intTriggerCapturedValue[0];
	if (sensor == LAPPING_SENSOR){
		return sensorVal->values.input0;
	}
	if (sensor == CREEL_SLIVER_CUT_SENSOR){
		return sensorVal->values.input1;
	}

	return -1;
}


void LappingSensorMonitor(SensorTypeDef *s){
	if (s->lappingSensor != 0){
		s->lappingTimerIncrementBool = 1;
		if (s->lappingSensorTimer >= 2){
			s->lappingTimerIncrementBool = 0;
			s->lappingSensorTimer = 0;
			s->lappingSensorOneShot = 1;
		}
	}
	else{
		s->lappingTimerIncrementBool = 0;
		s->lappingSensorTimer = 0;
	}
}


void SliverSensorMonitor(SensorTypeDef *s){
	if (s->sliverCutSensor == 0){
		s->sliverCutTimerIncrementBool = 1;
		if (s->slivercutSensorTimer >= 1){
			s->sliverCutTimerIncrementBool = 0;
			s->slivercutSensorTimer = 0;
			s->sliverCutOneShot = 1;
		}
	}
	else{
		s->sliverCutTimerIncrementBool = 0;
		s->slivercutSensorTimer = 0;
	}
}


int8_t Sensor_ReadValueDirectly(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor){
	mcp23017_read(mcp, MCP_GPIOB,mcp->intTriggerCapturedValue); // captures GPIO value when interrupt comes.
	sensorVal->raw = mcp->intTriggerCapturedValue[0];
	if (sensor == LAPPING_SENSOR){
		return sensorVal->values.input0;
	}
	if (sensor == CREEL_SLIVER_CUT_SENSOR){
		return sensorVal->values.input1;
	}
	return -1;
}

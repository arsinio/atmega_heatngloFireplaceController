/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#include "ovr_pulseTrainCommon.h"


// ******** includes ********


// ******** local macro definitions ********
#define PRESCALAR				TIMER_PRESCALE_8


// ******** local type definitions ********


// ******** local function prototypes ********


// ********  local variable declarations *********



// ******** global function implementations ********
uint8_t ovr_pulseTrainCommon_getPrescalarBits(timer_prescalar_t prescalarIn)
{
	return (uint8_t)prescalarIn;
}


float ovr_pulseTrainCommon_getTimerPeriod_ns(timer_prescalar_t prescalarIn)
{
	float retVal = 0.0;
	switch( prescalarIn )
	{
		case TIMER_PRESCALE_1:
			retVal = (1.0 / (((float)F_CPU) / 1.0)) * 1.0E9;
			break;

		case TIMER_PRESCALE_8:
			retVal = (1.0 / (((float)F_CPU) / 8.0)) * 1.0E9;
			break;

		case TIMER_PRESCALE_64:
			retVal = (1.0 / (((float)F_CPU) / 64.0)) * 1.0E9;
			break;

		case TIMER_PRESCALE_256:
			retVal = (1.0 / (((float)F_CPU) / 256.0)) * 1.0E9;
			break;

		case TIMER_PRESCALE_1024:
			retVal = (1.0 / (((float)F_CPU) / 1024.0)) * 1.0E9;
			break;
	}
	return retVal;
}


// ******** local function implementations ********

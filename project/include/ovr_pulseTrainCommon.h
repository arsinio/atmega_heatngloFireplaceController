/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#ifndef OVR_PULSETRAINCOMMON_H_
#define OVR_PULSETRAINCOMMON_H_


// ******** includes ********
#include <stdint.h>


// ******** global macro definitions ********
#define PULSE_TRAIN_COMMON_PRESCALAR				TIMER_PRESCALE_8


// ******** global type definitions *********
typedef enum
{
	TIMER_PRESCALE_1 = 0x01,
	TIMER_PRESCALE_8 = 0x02,
	TIMER_PRESCALE_64 = 0x03,
	TIMER_PRESCALE_256 = 0x04,
	TIMER_PRESCALE_1024 = 0x05
}timer_prescalar_t;


// ******** global function prototypes ********
uint8_t ovr_pulseTrainCommon_getPrescalarBits(timer_prescalar_t prescalarIn);
float ovr_pulseTrainCommon_getTimerPeriod_ns(timer_prescalar_t prescalarIn);

#endif

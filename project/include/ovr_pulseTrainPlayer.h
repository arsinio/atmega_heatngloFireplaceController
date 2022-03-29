/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#ifndef OVR_PULSETRAINPLAYER_H_
#define OVR_PULSETRAINPLAYER_H_


// ******** includes ********
#include <cxa_ioStream.h>


// ******** global macro definitions ********


// ******** global type definitions *********
typedef void (*ovr_pulseTrainPlayer_outputComplete_cb_t)(void* userVarIn);


// ******** global function prototypes ********
void ovr_pulseTrainPlayer_init(int threadIdIn, cxa_ioStream_t* iosIn);
void ovr_pulseTrainPlayer_outputPulseTrain(bool initialGpioStateIn,
										   const uint16_t* pulsePeriods_ticksIn, size_t numPulsePeriodsIn,
										   ovr_pulseTrainPlayer_outputComplete_cb_t cbIn, void* userVarIn);
bool ovr_pulseTrainPlayer_isPulseTrainInProgress(void);


#endif

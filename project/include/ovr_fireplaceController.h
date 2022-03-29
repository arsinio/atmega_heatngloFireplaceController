/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#ifndef OVR_FIREPLACECONTROLLER_H_
#define OVR_FIREPLACECONTROLLER_H_


// ******** includes ********
#include <cxa_adcChannel.h>
#include <cxa_gpio.h>
#include <cxa_ioStream.h>


// ******** global macro definitions ********


// ******** global type definitions *********


// ******** global function prototypes ********
void ovr_fireplaceController_init(int threadIdIn,
								  cxa_adcChannel_t* flameAdcIn,
								  cxa_adcChannel_t* fanAdcIn,
								  cxa_gpio_t* gpio_statusIn,
								  cxa_ioStream_t* ios_debugIn);

#endif

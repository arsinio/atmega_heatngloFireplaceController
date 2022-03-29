/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#include "ovr_pulseTrainPlayer.h"


// ******** includes ********
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <cxa_array.h>
#include <cxa_atmega_gpio.h>
#include <cxa_delay.h>
#include <cxa_runLoop.h>
#include <cxa_stateMachine.h>
#include <cxa_timeDiff.h>

#include <ovr_pulseTrainCommon.h>


#define CXA_LOG_LEVEL						CXA_LOG_LEVEL_DEBUG
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********


// ******** local type definitions ********


// ******** local function prototypes ********
static void runLoop_updateCb(void* userVarIn);


// ********  local variable declarations *********
static cxa_logger_t logger;
static cxa_atmega_gpio_t gpio_output;

static cxa_ioStream_t *ios_start;

static bool initialGpioState = 0;
static const uint16_t* pulsePeriods_ticks;
static size_t numPulsePeriods;
static size_t currPulsePeriod;

static bool hasCalledCallback = true;
static ovr_pulseTrainPlayer_outputComplete_cb_t outputCompleteCb;
static void* userVar;


// ******** global function implementations ********
void ovr_pulseTrainPlayer_init(int threadIdIn, cxa_ioStream_t* iosIn)
{
	// save our inputs
	ios_start = iosIn;

	cxa_logger_init(&logger, "pulsePlay");

	// register for runloop execution
	cxa_runLoop_addEntry(threadIdIn, NULL, runLoop_updateCb, NULL);
}


void ovr_pulseTrainPlayer_outputPulseTrain(bool initialGpioStateIn,
										   const uint16_t* pulsePeriods_ticksIn, size_t numPulsePeriodsIn,
										   ovr_pulseTrainPlayer_outputComplete_cb_t cbIn, void* userVarIn)
{
	// simple case
	if( numPulsePeriodsIn == 0 )
	{
		if( cbIn != NULL ) cbIn(userVarIn);
		return;
	}

	// save our inputs
	initialGpioState = initialGpioStateIn;
	pulsePeriods_ticks = pulsePeriods_ticksIn;
	numPulsePeriods = numPulsePeriodsIn;
	currPulsePeriod = 0;
	hasCalledCallback = false;
	outputCompleteCb = cbIn;
	userVar = userVarIn;

	cxa_logger_debug(&logger, "output waveform with %d periods", numPulsePeriods);
	cxa_delay_ms(100);

#if( CXA_LOG_LEVEL == CXA_LOG_LEVEL_TRACE )
	cxa_logger_trace(&logger, "expected periods (ns) follow");
	for( size_t i = 0; i < numPulsePeriods; i++)
	{
		uint16_t currTicks = pgm_read_word(&pulsePeriods_ticksIn[i]);
		float time_ns = ovr_pulseTrainCommon_getTimerPeriod_ns(ovr_pulseTrainCommon_getPrescalarBits(PULSE_TRAIN_COMMON_PRESCALAR)) * currTicks;

		cxa_ioStream_writeFormattedString(ios_start, "%lu", (uint32_t)time_ns);
		if( i != (numPulsePeriods-1) ) cxa_ioStream_writeString(ios_start, ", ");
		if( ((i+1) % 8) == 0) cxa_ioStream_writeLine(ios_start, "");
		cxa_delay_ms(25);
	}
	cxa_ioStream_writeLine(ios_start, "");
#endif


	// disable our interrupts
	cli();
	TIMSK1 &= ~(1 << 1);

	// make sure our timer is in a known state (and stopped)
	TCCR1A = 0;
	TCCR1B = 0;

	// initialize our GPIO to our first known state, give it a second to settle
	cxa_atmega_gpio_init_output(&gpio_output, PORTB, 1, CXA_GPIO_POLARITY_NONINVERTED, initialGpioStateIn);
	cxa_delay_ms(100);

	// CTC mode, toggle output on compare match, timer stopped
	TCCR1A |= (1 << 6);
	TCCR1B |= (1 << 3);

	// reset our timer
	TCNT1 = 0;
	OCR1A = pgm_read_word(&pulsePeriods_ticks[currPulsePeriod++]);
	TIMSK1 |= (1 << 1);
	TIFR1 |= (1 << 1);

	// enable interrupts
	sei();

	// start the timer, do our first transition
//	cxa_atmega_gpio_init_output(&gpio_output, PORTB, 1, CXA_GPIO_POLARITY_NONINVERTED, !initialGpioStateIn);
	TCCR1C |= (1 << 7);
	TCCR1B |= ovr_pulseTrainCommon_getPrescalarBits(PULSE_TRAIN_COMMON_PRESCALAR);
}


bool ovr_pulseTrainPlayer_isPulseTrainInProgress(void)
{
	return !hasCalledCallback;
}


// ******** local function implementations ********
static void runLoop_updateCb(void* userVarIn)
{
	// check to see if we're done and we need to call the callback
	if( (currPulsePeriod == numPulsePeriods) && !hasCalledCallback )
	{
		hasCalledCallback = true;
		cxa_logger_debug(&logger, "done!");
		cxa_delay_ms(25);
		if( outputCompleteCb != NULL ) outputCompleteCb(userVarIn);
	}
}


// ******** interrupt implementations *******
ISR(TIMER1_COMPA_vect)
{
	// check to see if we're done
	if( currPulsePeriod == numPulsePeriods )
	{
		// we're done
		TCCR1A = 0;
		TCCR1B = 0;
		TIMSK1 &= ~(1 << 1);
	}
	else
	{
		// keep going
		OCR1A = pgm_read_word(&pulsePeriods_ticks[currPulsePeriod++]);
	}
}

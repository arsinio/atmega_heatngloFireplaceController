/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#include "ovr_pulseTrainRecorder.h"


// ******** includes ********
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <cxa_array.h>
#include <cxa_atmega_gpio.h>
#include <cxa_delay.h>
#include <cxa_stateMachine.h>
#include <cxa_timeDiff.h>

#include <ovr_pulseTrainCommon.h>

#define CXA_LOG_LEVEL						CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********
#define MAX_TRANSITIONS       	256
#define WAIT_COMPLETE_MS		1000


// ******** local type definitions ********
typedef enum
{
	STATE_WAIT_FOR_USER,
	STATE_SETUP,
	STATE_WAIT_FIRST_EDGE,
	STATE_CAPTURING,
	STATE_WAIT_COMPLETE,
	STATE_COMPLETE
}state_t;


typedef enum
{
	CAPTURE_GOOD,
	CAPTURE_BAD_TIMER_OVERFLOW,
	CAPTURE_BAD_TOO_MANY_TRANSITIONS
}capture_status_t;


// ******** local function prototypes ********
static void disableInterrupts(void);
static void outputCaptureData(void);

static void stateCb_waitForUser_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn);
static void stateCb_waitForUser_state(cxa_stateMachine_t *const smIn, void* userVarIn);
static void stateCb_setup_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn);
static void stateCb_waitFirstEdge_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn);
static void stateCb_capturing_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn);
static void stateCb_waitComplete_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn);
static void stateCb_waitComplete_state(cxa_stateMachine_t *const smIn, void* userVarIn);
static void stateCb_captureComplete_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn);


// ********  local variable declarations *********
static cxa_logger_t logger;
static cxa_atmega_gpio_t gpio_input;
static cxa_stateMachine_t stateMachine;

static cxa_ioStream_t *ios_start;

static cxa_timeDiff_t td_waitComplete;

static bool initialGpioState = 0;
static capture_status_t captureStatus = CAPTURE_GOOD;

static cxa_array_t interruptTimes_ticks;
static uint16_t interruptTimes_ticks_raw[MAX_TRANSITIONS];


// ******** global function implementations ********
void ovr_pulseTrainRecorder_init(int threadIdIn, cxa_ioStream_t* iosIn)
{
	// save our inputs
	ios_start = iosIn;

	cxa_logger_init(&logger, "pulseRec");

	cxa_array_initStd(&interruptTimes_ticks, interruptTimes_ticks_raw);
	cxa_timeDiff_init(&td_waitComplete);

	// setup our state machine
	cxa_stateMachine_init(&stateMachine, "pulseTrainRecorder", threadIdIn);
	cxa_stateMachine_addState(&stateMachine, STATE_WAIT_FOR_USER, "waitForUser", stateCb_waitForUser_enter, stateCb_waitForUser_state, NULL, NULL);
	cxa_stateMachine_addState(&stateMachine, STATE_SETUP, "setup", stateCb_setup_enter, NULL, NULL, NULL);
	cxa_stateMachine_addState(&stateMachine, STATE_WAIT_FIRST_EDGE, "waitFirstEdge", stateCb_waitFirstEdge_enter, NULL, NULL, NULL);
	cxa_stateMachine_addState(&stateMachine, STATE_CAPTURING, "capturing", stateCb_capturing_enter, NULL, NULL, NULL);
	cxa_stateMachine_addState(&stateMachine, STATE_WAIT_COMPLETE, "waitComplete", stateCb_waitComplete_enter, stateCb_waitComplete_state, NULL, NULL);
	cxa_stateMachine_addState(&stateMachine, STATE_COMPLETE, "complete", stateCb_captureComplete_enter, NULL, NULL, NULL);
	cxa_stateMachine_setInitialState(&stateMachine, STATE_WAIT_FOR_USER);

	// setup our GPIO so we can get a good initial state (to determine initial edge)
	cxa_atmega_gpio_init_input(&gpio_input, PORTB, 0, CXA_GPIO_POLARITY_NONINVERTED);
}


// ******** local function implementations ********
static void disableInterrupts(void)
{
	// disable our interrupts
	cli();
	TIMSK1 &= ~((1 << 5) | (1 << 2));
	TIFR1 |= (1 << 5) | (1 << 2);
	sei();
}


static void outputCaptureData()
{
	uint16_t maxDiff_ticks = 0;

	cxa_logger_debug(&logger, "%d transitions", cxa_array_getSize_elems(&interruptTimes_ticks));
	cxa_delay_ms(100);
	cxa_ioStream_writeLine(ios_start, "");
	cxa_ioStream_writeFormattedLine(ios_start, "bool initialGpioState = %d;", initialGpioState);
	cxa_delay_ms(100);
	cxa_ioStream_writeLine(ios_start, "const uint16_t pulsePeriods_ticks[] PROGMEM = {");
	cxa_delay_ms(100);
	for( size_t i = 0; i < cxa_array_getSize_elems(&interruptTimes_ticks)-1; i++ )
	{
		uint16_t edge1_ticks = *(uint16_t*)cxa_array_get(&interruptTimes_ticks, i);
		uint16_t edge2_ticks = *(uint16_t*)cxa_array_get(&interruptTimes_ticks, i+1);

		uint16_t diff_ticks = edge2_ticks - edge1_ticks;
		if( diff_ticks > maxDiff_ticks ) maxDiff_ticks = diff_ticks;

//		uint32_t diff_ticks_32 = diff_ticks & 0x0000FFFF;
//		float time_ns = ovr_pulseTrainCommon_getTimerPeriod_ns(TCCR1B & 0x07) * diff_ticks_32;
		cxa_ioStream_writeFormattedString(ios_start, "%u", diff_ticks);
		if( i != cxa_array_getSize_elems(&interruptTimes_ticks)-2 ) cxa_ioStream_writeString(ios_start, ", ");
		if( ((i+1) % 8) == 0) cxa_ioStream_writeLine(ios_start, "");
		cxa_delay_ms(25);
	}
	cxa_ioStream_writeLine(ios_start, "");
	cxa_ioStream_writeLine(ios_start, "};");
	cxa_ioStream_writeLine(ios_start, "");

	if( maxDiff_ticks > 0 ) cxa_logger_debug(&logger, "max ticks: %d", maxDiff_ticks);
}


static void stateCb_waitForUser_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn)
{
	cxa_logger_debug(&logger, "press any key to continue");
	cxa_delay_ms(100);
}


static void stateCb_waitForUser_state(cxa_stateMachine_t *const smIn, void* userVarIn)
{
	if( cxa_ioStream_readByte(ios_start, NULL) == CXA_IOSTREAM_READSTAT_GOTDATA )
	{
		cxa_stateMachine_transition(&stateMachine, STATE_SETUP);
	}
}


static void stateCb_setup_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn)
{
	// disable our interrupts
	cli();
	TIMSK1 &= ~(1 << 5);
	TIMSK1 &= ~(1 << 2);

	// reset our timer control register to a known-good state
	TCCR1A = 0;
	TCCR1B = 0;

	// set our prescalar
	TCCR1B |= ovr_pulseTrainCommon_getPrescalarBits(PULSE_TRAIN_COMMON_PRESCALAR);

	// clear previous captures
	cxa_array_clear(&interruptTimes_ticks);

	// get our initial GPIO state
	if( (initialGpioState = cxa_gpio_getValue(&gpio_input.super)) )
	{
		// we're currently high...arm falling edge
		cxa_logger_debug(&logger, "arming falling edge");
		TCCR1B &= ~(1 << 6);
	}
	else
	{
		// we're currently low...arm rising edge
		cxa_logger_debug(&logger, "arming rising edge");
		TCCR1B |= (1 << 6);
	}

	// mark our capture status as good (until something bad happens)
	captureStatus = CAPTURE_GOOD;

	// enable interrupts (just input capture for now)
	TIFR1 |= (1 << 5);
	TIMSK1 = (1 << 5);
	sei();

	// ready for our first edge
	cxa_stateMachine_transition(&stateMachine, STATE_WAIT_FIRST_EDGE);
}


static void stateCb_waitFirstEdge_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn)
{
	cxa_logger_debug(&logger, "waiting for first edge");
}


static void stateCb_capturing_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn)
{
	cxa_logger_debug(&logger, "capturing");
}


static void stateCb_waitComplete_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn)
{
	cxa_logger_debug(&logger, "wait for complete");

	cxa_timeDiff_setStartTime_now(&td_waitComplete);
}


static void stateCb_waitComplete_state(cxa_stateMachine_t *const smIn, void* userVarIn)
{
	if( cxa_timeDiff_isElapsed_ms(&td_waitComplete, WAIT_COMPLETE_MS) )
	{
		// we waited a bit and we didn't have any other transitions...we're done!
		captureStatus = CAPTURE_GOOD;
		cxa_stateMachine_transition(&stateMachine, STATE_COMPLETE);
	}
}


static void stateCb_captureComplete_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn)
{
	// make sure interrupts are disabled
	disableInterrupts();

	switch( captureStatus )
	{
		case CAPTURE_GOOD:
			cxa_logger_debug(&logger, "capture good, %d transitions", cxa_array_getSize_elems(&interruptTimes_ticks));
			outputCaptureData();
			break;

		case CAPTURE_BAD_TIMER_OVERFLOW:
			cxa_logger_debug(&logger, "bad capture, timer overflow");
			break;

		case CAPTURE_BAD_TOO_MANY_TRANSITIONS:
			cxa_logger_debug(&logger, "bad capture, too many transitions");
			break;
	}

	cxa_delay_ms(100);
	cxa_stateMachine_transition(&stateMachine, STATE_WAIT_FOR_USER);
}


// ******** interrupt implementations *******
ISR(TIMER1_CAPT_vect)
{
	// immediately switch the edge so we don't miss any interrupts
	TIFR1 |= (1 << 5);
	TCCR1B ^= (1 << 6);

	// see if we're waiting for the capture to be complete
	// if we were, this means we've already overflowed our timer
	switch( cxa_stateMachine_getCurrentState(&stateMachine) )
	{
		case STATE_SETUP:
			// do nothing
			return;

		case STATE_WAIT_FIRST_EDGE:
			// continue, but mark our state as changed
			cxa_stateMachine_transition(&stateMachine, STATE_CAPTURING);
			break;

		case STATE_CAPTURING:
			// continue
			break;

		case STATE_WAIT_COMPLETE:
			disableInterrupts();
			captureStatus = CAPTURE_BAD_TIMER_OVERFLOW;
			cxa_stateMachine_transition(&stateMachine, STATE_COMPLETE);
			return;

		case STATE_COMPLETE:
			// do nothing
			return;
	}
	// if we made it here, we're in a good state to record the capture

	// record our time and change our edge detection
	uint16_t time_ticks = ICR1;

	// record our interrupt time
	if( !cxa_array_append(&interruptTimes_ticks, &time_ticks) )
	{
		disableInterrupts();
		captureStatus = CAPTURE_BAD_TOO_MANY_TRANSITIONS;
		cxa_stateMachine_transition(&stateMachine, STATE_COMPLETE);
		return;
	}

	// setup our timer comparator to catch overflows and enable interrupts
	OCR1B = time_ticks - 1;
	TIFR1 |= (1 << 2);
	TIMSK1 |= (1 << 2);

	// if we made it here, we're good to continue capturing...
}


ISR(TIMER1_COMPB_vect)
{
	// this should only happen once per capture...disable future interrupts
	TIMSK1 &= ~(1 << 2);

	// this isn't necessarily a bad thing, it could just mean that the waveform has ended
	// the first time this interrupt occurs, flag the capture as possibly complete
	// then wait around for another second or so to see if we get any more edges
	// if we _do_ get more edges, then we had an overflow...of we don't get anymore
	// edges, then it _was_ the end of the capture
	cxa_stateMachine_transition(&stateMachine, STATE_WAIT_COMPLETE);
}

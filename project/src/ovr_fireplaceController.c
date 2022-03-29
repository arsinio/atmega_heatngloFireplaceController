/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#include "ovr_fireplaceController.h"


// ******** includes ********
#include <cxa_assert.h>
#include <cxa_delay.h>
#include <cxa_stateMachine.h>
#include <cxa_timeBase.h>
#include <cxa_timeDiff.h>
#include "ovr_fireplaceController_pulseTrainCaptures.h"

#include <ovr_pulseTrainPlayer.h>

#define CXA_LOG_LEVEL				CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********
#define PRINT_STATUS_PERIOD_MS			1000
#define TRANSITION_HOLDOFF_PERIOD_MS	1000

#define AVG_BUFFER_LEN					92
#define AVG_WINDOW_RAW					8

#define FLAME_VAL_OFF_RAW				0
#define FLAME_VAL_OFF_LOW_RAW			94
#define FLAME_VAL_LOW_HIGH_RAW			273
#define FLAME_VAL_HIGH_RAW				UINT16_MAX

#define FAN_VAL_OFF_RAW					0
#define FAN_VAL_OFF_LOW_RAW				156
#define FAN_VAL_LOW_HIGH_RAW			1023
#define FAN_VAL_HIGH_RAW				UINT16_MAX

#define PERIODS_ARRAY_AND_SIZE(x)		x, (sizeof(x)/sizeof(*x))


// ******** local type definitions ********
typedef enum
{
	STATE_PRIME_ADCS,

	STATE_FLAME_OFF_FAN_OFF,
	STATE_FLAME_LOW_FAN_OFF,
	STATE_FLAME_HIGH_FAN_OFF,

	STATE_FLAME_OFF_FAN_LOW,
	STATE_FLAME_LOW_FAN_LOW,
	STATE_FLAME_HIGH_FAN_LOW,

	STATE_FLAME_OFF_FAN_HIGH,
	STATE_FLAME_LOW_FAN_HIGH,
	STATE_FLAME_HIGH_FAN_HIGH,
}state_t;


typedef struct
{
	state_t state;

	const uint16_t* pulseTrain_periods_ticks;
	size_t numPulseTrainPeriods;

	uint16_t flame_adcMinVal_raw;
	uint16_t flame_adcMaxVal_raw;

	uint16_t fan_adcMinVal_raw;
	uint16_t fan_adcMaxVal_raw;

	const char name[20];
}state_info_entry_t;


static state_info_entry_t stateInfo[] =
{
	{STATE_FLAME_OFF_FAN_OFF,   PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameOff_fanOff),   FLAME_VAL_OFF_RAW,      FLAME_VAL_OFF_LOW_RAW-1,  FAN_VAL_OFF_RAW, FAN_VAL_OFF_LOW_RAW-1,      "flameOff-fanOff"},
	{STATE_FLAME_LOW_FAN_OFF,   PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameLow_fanOff),   FLAME_VAL_OFF_LOW_RAW,  FLAME_VAL_LOW_HIGH_RAW-1, FAN_VAL_OFF_RAW, FAN_VAL_OFF_LOW_RAW-1,      "flameLow-fanOff"},
	{STATE_FLAME_HIGH_FAN_OFF,  PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameHigh_fanOff),  FLAME_VAL_LOW_HIGH_RAW, FLAME_VAL_HIGH_RAW,       FAN_VAL_OFF_RAW, FAN_VAL_OFF_LOW_RAW-1,      "flameHigh-fanOff"},

	{STATE_FLAME_OFF_FAN_LOW,   PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameOff_fanLow),   FLAME_VAL_OFF_RAW,      FLAME_VAL_OFF_LOW_RAW-1,  FAN_VAL_OFF_LOW_RAW, FAN_VAL_LOW_HIGH_RAW-1, "flameOff-fanLow"},
	{STATE_FLAME_LOW_FAN_LOW,   PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameLow_fanLow),   FLAME_VAL_OFF_LOW_RAW,  FLAME_VAL_LOW_HIGH_RAW-1, FAN_VAL_OFF_LOW_RAW, FAN_VAL_LOW_HIGH_RAW-1, "flameLow-fanLow"},
	{STATE_FLAME_HIGH_FAN_LOW,  PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameHigh_fanLow),  FLAME_VAL_LOW_HIGH_RAW, FLAME_VAL_HIGH_RAW,       FAN_VAL_OFF_LOW_RAW, FAN_VAL_LOW_HIGH_RAW-1, "flameHigh-fanLow"},

	{STATE_FLAME_OFF_FAN_HIGH,  PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameOff_fanHigh),  FLAME_VAL_OFF_RAW,      FLAME_VAL_OFF_LOW_RAW-1,  FAN_VAL_LOW_HIGH_RAW, FAN_VAL_HIGH_RAW,      "flameOff-fanHigh"},
	{STATE_FLAME_LOW_FAN_HIGH,  PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameLow_fanHigh),  FLAME_VAL_OFF_LOW_RAW,  FLAME_VAL_LOW_HIGH_RAW-1, FAN_VAL_LOW_HIGH_RAW, FAN_VAL_HIGH_RAW,      "flameLow-fanHigh"},
	{STATE_FLAME_HIGH_FAN_HIGH, PERIODS_ARRAY_AND_SIZE(pulsePeriods_ticks_flameHigh_fanHigh), FLAME_VAL_LOW_HIGH_RAW, FLAME_VAL_HIGH_RAW,       FAN_VAL_LOW_HIGH_RAW, FAN_VAL_HIGH_RAW,      "flameHigh-fanHigh"},
};


// ******** local function prototypes ********
static state_info_entry_t* getInfoForState(state_t stateIn);
static state_t getStateForAdcValues(void);

static void stateCb_primeAdcs_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn);
static void stateCb_primeAdcs_state(cxa_stateMachine_t *const smIn, void* userVarIn);

static void stateCb_all_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn);
static void stateCb_all_state(cxa_stateMachine_t *const smIn, void* userVarIn);

static void adcConversionCb_flame(cxa_adcChannel_t *const adcChanIn, bool wasSuccessfulIn, float readVoltageIn, uint16_t rawValueIn, void* userVarIn);
static void adcConversionCb_fan(cxa_adcChannel_t *const adcChanIn, bool wasSuccessfulIn, float readVoltageIn, uint16_t rawValueIn, void* userVarIn);


// ********  local variable declarations *********
static cxa_adcChannel_t* flameAdc;
static cxa_adcChannel_t* fanAdc;

static uint16_t adcPrimeCount = 0;
static float avg_adcVal_flame = 0;
static float avg_topAdcVal_flame = 0;
static float avg_botAdcVal_flame = 0;

static float avg_adcVal_fan = 0;
static float avg_topAdcVal_fan = 0;
static float avg_botAdcVal_fan = 0;

static cxa_gpio_t* gpio_status;
static cxa_timeDiff_t td_printStatus;

static cxa_timeDiff_t td_transitionHoldoff;

static cxa_ioStream_t* ios_debug;
static bool isOverrideActive = false;

static cxa_stateMachine_t stateMachine;
static cxa_logger_t logger;


// ******** global function implementations ********
void ovr_fireplaceController_init(int threadIdIn,
								  cxa_adcChannel_t* flameAdcIn,
								  cxa_adcChannel_t* fanAdcIn,
								  cxa_gpio_t* gpio_statusIn,
								  cxa_ioStream_t* ios_debugIn)
{
	cxa_assert(flameAdcIn);
	cxa_assert(fanAdcIn);
	cxa_assert(gpio_statusIn);

	// store our references
	flameAdc = flameAdcIn;
	fanAdc = fanAdcIn;
	gpio_status = gpio_statusIn;
	ios_debug = ios_debugIn;

	// setup our logger
	cxa_logger_init(&logger, "fireCntrl");
	cxa_timeDiff_init(&td_printStatus);
	cxa_timeDiff_init(&td_transitionHoldoff);

	// setup our state machine
	cxa_stateMachine_init(&stateMachine, "fpCont", threadIdIn);
	cxa_stateMachine_addState(&stateMachine, STATE_PRIME_ADCS, "primeAdcs", stateCb_primeAdcs_enter, stateCb_primeAdcs_state, NULL, NULL);
	for( int i = 0; i < sizeof(stateInfo)/sizeof(*stateInfo); i++ )
	{
		cxa_stateMachine_addState(&stateMachine, stateInfo[i].state, stateInfo[i].name, stateCb_all_enter, stateCb_all_state, NULL, NULL);
	}
	cxa_stateMachine_setInitialState(&stateMachine, STATE_PRIME_ADCS);

	// register for adc conversions
	cxa_adcChannel_addListener(flameAdc, adcConversionCb_flame, NULL);
	cxa_adcChannel_addListener(fanAdc, adcConversionCb_fan, NULL);


	DDRD |= (1 << 2);
}


// ******** local function implementations ********
static state_info_entry_t* getInfoForState(state_t stateIn)
{
	for( int i = 0; i < sizeof(stateInfo)/sizeof(*stateInfo); i++ )
	{
		if( stateInfo[i].state == stateIn ) return &stateInfo[i];
	}
	return NULL;
}


static state_t getStateForAdcValues()
{
	uint16_t adcPeakToPeak_flame_raw = ((uint16_t)avg_topAdcVal_flame) - ((uint16_t)avg_botAdcVal_flame);
	uint16_t adcPeakToPeak_fan_raw = ((uint16_t)avg_topAdcVal_fan) - ((uint16_t)avg_botAdcVal_fan);

	for( int i = 0; i < sizeof(stateInfo)/sizeof(*stateInfo); i++ )
	{
		if( (stateInfo[i].flame_adcMinVal_raw <= adcPeakToPeak_flame_raw) && (adcPeakToPeak_flame_raw <= stateInfo[i].flame_adcMaxVal_raw) &&
			(stateInfo[i].fan_adcMinVal_raw <= adcPeakToPeak_fan_raw) && (adcPeakToPeak_fan_raw <= stateInfo[i].fan_adcMaxVal_raw))
		{
			return stateInfo[i].state;
		}
	}
	// default is to turn off
	return STATE_FLAME_OFF_FAN_OFF;
}


static void stateCb_primeAdcs_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn)
{
	cxa_logger_debug(&logger, "priming ADCs");
}


static void stateCb_primeAdcs_state(cxa_stateMachine_t *const smIn, void* userVarIn)
{
	// check our ADCs...start with the flame ADC, then chain to the fan ADC
	cxa_adcChannel_startConversion_singleShot(flameAdc);
}


static void stateCb_all_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void* userVarIn)
{
	// get our state info
	state_info_entry_t* currStateInfo = getInfoForState(cxa_stateMachine_getCurrentState(&stateMachine));
	if( currStateInfo == NULL ) return;

	cxa_logger_debug(&logger, "newState: %s", currStateInfo->name);
	cxa_delay_ms(50);

	// send our pulse train
	ovr_pulseTrainPlayer_outputPulseTrain(0, currStateInfo->pulseTrain_periods_ticks, currStateInfo->numPulseTrainPeriods, NULL, userVarIn);

	// reset our transition holdoff delay
	cxa_timeDiff_setStartTime_now(&td_transitionHoldoff);
}


static void stateCb_all_state(cxa_stateMachine_t *const smIn, void* userVarIn)
{
	// check to see if we have a serial console override
	uint8_t readByte;
	if( cxa_ioStream_readByte(ios_debug, &readByte) == CXA_IOSTREAM_READSTAT_GOTDATA )
	{
		switch( readByte )
		{
			case '1':
				isOverrideActive = true;
				cxa_stateMachine_transition(&stateMachine, STATE_FLAME_OFF_FAN_OFF);
				break;

			case '2':
				isOverrideActive = true;
				cxa_stateMachine_transition(&stateMachine, STATE_FLAME_LOW_FAN_OFF);
				break;

			case '3':
				isOverrideActive = true;
				cxa_stateMachine_transition(&stateMachine, STATE_FLAME_HIGH_FAN_OFF);
				break;

			case '0':
				isOverrideActive = false;
				break;
		}
		return;
	}

	if( !isOverrideActive )
	{
		// check our ADCs...start with the flame ADC, then chain to the fan ADC
		cxa_adcChannel_startConversion_singleShot(flameAdc);
	}
}


static void adcConversionCb_flame(cxa_adcChannel_t *const adcChanIn, bool wasSuccessfulIn, float readVoltageIn, uint16_t rawValueIn, void* userVarIn)
{
	// record our average flame value
	avg_adcVal_flame = avg_adcVal_flame - (((float)avg_adcVal_flame) / ((float)AVG_BUFFER_LEN)) + (((float)rawValueIn) / ((float)AVG_BUFFER_LEN));
	if( rawValueIn > (avg_adcVal_flame + AVG_WINDOW_RAW) )
	{
		avg_topAdcVal_flame = avg_topAdcVal_flame - (((float)avg_topAdcVal_flame) / ((float)AVG_BUFFER_LEN)) + (((float)rawValueIn) / ((float)AVG_BUFFER_LEN));
	}
	else if( rawValueIn < (avg_adcVal_flame - AVG_WINDOW_RAW) )
	{
		avg_botAdcVal_flame = avg_botAdcVal_flame - (((float)avg_botAdcVal_flame) / ((float)AVG_BUFFER_LEN)) + (((float)rawValueIn) / ((float)AVG_BUFFER_LEN));
	}
	else
	{
		avg_topAdcVal_flame = avg_topAdcVal_flame - (((float)avg_topAdcVal_flame) / ((float)AVG_BUFFER_LEN)) + (((float)avg_adcVal_flame) / ((float)AVG_BUFFER_LEN));
		avg_botAdcVal_flame = avg_botAdcVal_flame - (((float)avg_botAdcVal_flame) / ((float)AVG_BUFFER_LEN)) + (((float)avg_adcVal_flame) / ((float)AVG_BUFFER_LEN));
	}

	// chain to our fan ADC call
	cxa_adcChannel_startConversion_singleShot(fanAdc);
}


static void adcConversionCb_fan(cxa_adcChannel_t *const adcChanIn, bool wasSuccessfulIn, float readVoltageIn, uint16_t rawValueIn, void* userVarIn)
{
	// record our average fan value
	avg_adcVal_fan = avg_adcVal_fan - (((float)avg_adcVal_fan) / ((float)AVG_BUFFER_LEN)) + (((float)rawValueIn) / ((float)AVG_BUFFER_LEN));
	if( rawValueIn > (avg_adcVal_fan + AVG_WINDOW_RAW) )
	{
		avg_topAdcVal_fan = avg_topAdcVal_fan - (((float)avg_topAdcVal_fan) / ((float)AVG_BUFFER_LEN)) + (((float)rawValueIn) / ((float)AVG_BUFFER_LEN));
	}
	else if( rawValueIn < (avg_adcVal_fan - AVG_WINDOW_RAW) )
	{
		avg_botAdcVal_fan = avg_botAdcVal_fan - (((float)avg_botAdcVal_fan) / ((float)AVG_BUFFER_LEN)) + (((float)rawValueIn) / ((float)AVG_BUFFER_LEN));
	}
	else
	{
		avg_topAdcVal_fan = avg_topAdcVal_fan - (((float)avg_topAdcVal_fan) / ((float)AVG_BUFFER_LEN)) + (((float)avg_adcVal_fan) / ((float)AVG_BUFFER_LEN));
		avg_botAdcVal_fan = avg_botAdcVal_fan - (((float)avg_botAdcVal_fan) / ((float)AVG_BUFFER_LEN)) + (((float)avg_adcVal_fan) / ((float)AVG_BUFFER_LEN));
	}


	// make sure we're not in the middle of outputting a pulse train
	if( ovr_pulseTrainPlayer_isPulseTrainInProgress() ) return;

	// see if we're priming the ADCs (boot)
	state_t currState = cxa_stateMachine_getCurrentState(&stateMachine);
	if( currState == STATE_PRIME_ADCS )
	{
		adcPrimeCount++;
		if( adcPrimeCount < (AVG_BUFFER_LEN * 10) )
		{
			// still priming ADCs
			return;
		}
	}
	// if we made it here, we're fully operational


	// print our status occasionally
	if( cxa_timeDiff_isElapsed_recurring_ms(&td_printStatus, PRINT_STATUS_PERIOD_MS) )
	{
		state_info_entry_t* currStateInfo = getInfoForState(currState);
		if( currStateInfo == NULL ) return;
		cxa_logger_debug(&logger, "%s  flamePP: %d   fanPP: %d",
				currStateInfo->name,
				(uint16_t)avg_topAdcVal_flame - (uint16_t)avg_botAdcVal_flame,
				(uint16_t)avg_topAdcVal_fan - (uint16_t)avg_botAdcVal_fan);
		cxa_gpio_toggle(gpio_status);
		cxa_delay_ms(50);
	}


	// make our decision about the next state
	state_t nextState = getStateForAdcValues();
	if( currState != nextState )
	{
		// need to change states...make the system wait at least a little bit to make sure we don't transition states too fast
		if( cxa_timeDiff_isElapsed_ms(&td_transitionHoldoff, TRANSITION_HOLDOFF_PERIOD_MS) )
		{
			cxa_stateMachine_transition(&stateMachine, nextState);
		}
	}
}

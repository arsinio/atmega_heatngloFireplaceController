/**
 * @author Christopher Armenio
 */


// ******** includes ********
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <cxa_assert.h>
#include <cxa_atmega_adcChannel.h>
#include <cxa_atmega_gpio.h>
#include <cxa_atmega_timeBase.h>
#include <cxa_atmega_timer8.h>
#include <cxa_atmega_usart.h>
#include <cxa_delay.h>
#include <cxa_fixedFifo.h>
#include <cxa_reboot.h>
#include <cxa_runLoop.h>
#include <cxa_timeDiff.h>

#include <ovr_pulseTrainPlayer.h>
#include <ovr_fireplaceController.h>
#include <util/delay.h>

#define CXA_LOG_LEVEL			CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********
#define THREAD_ID_MAIN					1


// ******** local type definitions ********


// ******** local function prototypes ********
static void sysInit();

static void assert_cb_onAssert(void);


// ********  local variable declarations *********
static cxa_atmega_adcChannel_t adc_flame;
static cxa_atmega_adcChannel_t adc_fan;

static cxa_atmega_gpio_t gpio_status;

static cxa_atmega_timer8_t timer_timeBase;
static cxa_atmega_usart_t usart_debug;




// ******** global function implementations ********
int main(void)
{
	cli();

	// disable system prescaler (clock will run at 16MHz external)
	CLKPR = _BV(CLKPCE);
	CLKPR = 0;

	sysInit();
	cxa_delay_ms(1000);
	sei();

	ovr_pulseTrainPlayer_init(THREAD_ID_MAIN, cxa_usart_getIoStream(&usart_debug.super));
	ovr_fireplaceController_init(THREAD_ID_MAIN, &adc_flame.super, &adc_fan.super, &gpio_status.super, cxa_usart_getIoStream(&usart_debug.super));

	cxa_timeDiff_t td;
	cxa_timeDiff_init(&td);

	while(1)
	{
		cxa_runLoop_iterate(THREAD_ID_MAIN);
	}
}


// ******** local function implementations ********
static void sysInit()
{
	cxa_assert_setAssertCb(assert_cb_onAssert);

	cxa_atmega_timer8_init(&timer_timeBase, CXA_ATM_TIMER8_0, CXA_ATM_TIMER8_MODE_FASTPWM, CXA_ATM_TIMER8_PRESCALE_1024);
	cxa_atmega_timeBase_initWithTimer8(&timer_timeBase);

	cxa_atmega_usart_init_noHH(&usart_debug, CXA_ATM_USART_ID_0, 9600);
	cxa_logger_setGlobalIoStream(cxa_usart_getIoStream(&usart_debug.super));
	cxa_assert_setIoStream(cxa_usart_getIoStream(&usart_debug.super));

	cxa_atmega_adcChannel_init(&adc_flame, 0, CXA_ATM_ADCCHAN_REF_AVCC);
	cxa_atmega_adcChannel_init(&adc_fan, 1, CXA_ATM_ADCCHAN_REF_AVCC);

	cxa_atmega_gpio_init_output(&gpio_status, PORTB, 5, CXA_GPIO_POLARITY_NONINVERTED, 0);
}



static void assert_cb_onAssert(void)
{
	cli();

	// ensure our timers are disconnected
	TCCR0A = 0;
	TCCR1A = 0;
	TCCR2A = 0;

	cxa_delay_ms(5000);
	cxa_reboot();
}

/*
 * cxa_config.h
 *
 * Created: 5/25/2015 8:55:08 PM
 *  Author: Christopher Armenio
 */

#ifndef CXA_CONFIG_H_
#define CXA_CONFIG_H_

#define CXA_ASSERT_EXIT_FUNC(eStat)					while(1);
//#define CXA_ASSERT_LINE_NUM_ENABLE
//#define CXA_ASSERT_MSG_ENABLE

#define CXA_FILE_DISABLE

#define CXA_I2CSLAVE_MAXNUM_COMMANDS				15
#define CXA_IOSTREAM_FORMATTED_BUFFERLEN_BYTES		80

#define CXA_LINE_ENDING								"\r\n"

//#define CXA_LOGGER_TIME_ENABLE

#define CXA_RUNLOOP_MAXNUM_ENTRIES 					4
#define CXA_RUNLOOP_INFOPRINT_PERIOD_MS				0

//#define CXA_STATE_MACHINE_ENABLE_LOGGING
//#define CXA_STATE_MACHINE_ENABLE_TIMED_STATES
#define CXA_STATE_MACHINE_MAXNUM_STATES				10

#endif /* CXA_CONFIG_H_ */

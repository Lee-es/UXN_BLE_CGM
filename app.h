/***************************************************************************//**
 * @brief app.h
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H_
#define APP_H_

#include "gecko_configuration.h"

/* DEBUG_LEVEL is used to enable/disable debug prints. Set DEBUG_LEVEL to 1 to enable debug prints */
#define DEBUG_LEVEL 1

/* Set this value to 1 if you want to disable deep sleep completely */
#define DISABLE_SLEEP 0

#if DEBUG_LEVEL
#include "retargetserial.h"
#include <stdio.h>
#endif

#if DEBUG_LEVEL
#define initLog()     RETARGET_SerialInit()
#define flushLog()    RETARGET_SerialFlush()
#define printLog(...) printf(__VA_ARGS__)
#else
#define initLog()
#define flushLog()
#define printLog(...)
#endif



#define _SODYKIM_CODE_
#define _DEBUG_MSG_

#ifdef _SODYKIM_CODE_

#define	ENABLE		1
#define	DISABLE		0

#define	MAX_GLUCOSE_SIZE	60
#define	MAX_PICK_SIZE		4

typedef struct {
	double pick_buff[MAX_PICK_SIZE];
	double calc_data;

	double calc_sum_data;

	uint16_t	calc_sum_count;
	uint16_t	dummy2;

	uint8_t	sum_count;
	uint8_t	sum_flag;
	uint8_t	pick_count;
	uint8_t	dummy1;


	uint16_t	send_timer;
	uint16_t	max_timer;

}glucose_t;

typedef struct{
	glucose_t		glucose;
} main_control_t;


extern main_control_t MainCon;
extern main_control_t *pMainCon;
#endif

/* Main application */
void appMain(gecko_configuration_t *pconfig);

#endif

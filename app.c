/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
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

/* Bluetooth stack headers */
#include <string.h>

#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "app.h"
#include "gpiointerrupt.h"
#include "em_i2c.h"
#include "em_gpio.h"

#include "lis2dh12_reg.h"
#include "slg47004.h"

#define EXT_SIGNAL_HALL_SENSOR      0x01

extern lis2dh12_ctx_t 	dev_ctx;
extern slg47004_ctx_t	op_amp_ctx;

//extern double read_sensor(void);
extern void calc_glucose_sensor(void);
extern void read_VDD(void);
extern void initIADC(void);

extern void init_GPIO(void);
extern void POWER_Enable(unsigned int out);
extern void RP517_Enable(unsigned int out);

extern uint32_t UTIL_init(void);
extern void delay_ms(uint32_t ms);

extern void init_motion(void);
extern void SLG47004_op_amp_set(void);

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

#define TIMER_ID_250MSEC					10
#define TIMER_ID_60SEC						20
#define TIMER_ID_1SEC						30

uint8_t stream_mode = 1;
uint8_t connected_with_mobile = 0;
uint8 szBuff[256];
uint8 len = 0;
static uint8 _conn_handle = 0xFF;

extern double singleResult_SENSOR;
extern double singleResult_Ref_Sensor;
extern double singleResult_VDD;

#ifdef _SODYKIM_CODE_
main_control_t MainCon;
main_control_t *pMainCon;

void device_control_attach(void)
{
	pMainCon = &MainCon;
	memset(pMainCon, 0, sizeof(main_control_t));
}

#endif


/**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device.
 */
void set_device_name(bd_addr *pAddr)
{
	char name[20];
	uint16 res;

	// create unique device name using the last two bytes of the Bluetooth address
	sprintf(name, "UXN_CGM %x:%x", pAddr->addr[1], pAddr->addr[0]);
#if 0
	printLog("\r\n\t------------------------------\r\n", name);
	printLog("\tDevice name: '%s'\r\n", name);
	printLog("\t------------------------------\r\n", name);
#endif

	res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
	if (res)
	{
		printLog("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
	}
}

/***************************************************************************//**
 * @brief  Gpio callback
 * @param  pin - pin which triggered interrupt
 ******************************************************************************/
void gpioCallback(uint8_t intNo)
{
	if (intNo == 03)
	{
		gecko_external_signal(EXT_SIGNAL_HALL_SENSOR);
	}
}

void GPIO_interrupt_enable(void)
{
	/* Initialize GPIO interrupt dispatcher */
	GPIOINT_Init();

#if 0
	/* Configure PC03 as input from Hall Sensor */
	GPIO_PinModeSet(gpioPortC, 03, gpioModeInput, 0);

	/* Set falling edge interrupt for both ports */
	GPIO_IntConfig(gpioPortC, 03, true, false, true);

	/* Register callbacks before setting up and enabling pin interrupt. */
	GPIOINT_CallbackRegister(03, gpioCallback);
#else
	/* Configure PC06 as input from Hall Sensor */
	GPIO_PinModeSet(gpioPortC, 05, gpioModeInput, 0);

	/* Set falling edge interrupt for both ports */
	GPIO_IntConfig(gpioPortC, 05, true, false, true);

	/* Register callbacks before setting up and enabling pin interrupt. */
	GPIOINT_CallbackRegister(05, gpioCallback);

#endif
}

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
	pconfig->sleep.flags = 0;
#endif

	struct gecko_msg_system_get_bt_address_rsp_t *pAddr;
	uint16 result;

#if 1

	/* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
	initLog();
	printLog("\r\n");
	printLog("##############################\r\n");
	printLog("#       UXN CGM PROJECT      #\r\n");
	printLog("##############################\r\n");
	printLog("\r\n");
#endif
	/* Initialize IADC */
	initIADC();

	/* Initialize GPIO Interrupt */
//	GPIO_interrupt_enable();

	/* Initialize Motion Sensor */
	init_motion();

	/***************************************/
	// GPIO init and enable interrupt
	/***************************************/
	GPIO_interrupt_enable();

	/* Initialize stack */
	gecko_init(pconfig);

	init_GPIO();

	UTIL_init();

#ifdef _SODYKIM_CODE_
	device_control_attach();
#endif

//	pMainCon->glucose.max_timer = 60*5;						// 60sec * 5 = 300sec			1분 간격--> 5min
//	pMainCon->glucose.max_timer = 60*5;						// 60sec						1초 간격
	pMainCon->glucose.max_timer = 2;						// 60sec						1초 간격
	pMainCon->glucose.send_timer= 0;

	while (1)
	{
		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;

		/* if there are no events pending then the next call to gecko_wait_event() may cause
		* device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
		if (!gecko_event_pending())
		{
			printLog("gecko_event_pending\r\n");
			flushLog();
		}

		/* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
		evt = gecko_wait_event();

		printLog("wake up for event\r\n");

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header))
		{
			/* This boot event is generated when the system boots up after reset.
			 * Do not call any stack commands before receiving the boot event.
			 * Here the system is set to start advertising immediately after boot procedure. */
			case gecko_evt_system_boot_id:

				pAddr = gecko_cmd_system_get_bt_address();
				set_device_name(&pAddr->address);

				bootMessage(&(evt->data.evt_system_boot));
				printLog("boot event - starting advertising\r\n");

				/* Set advertising parameters. 1000ms advertisement interval.
				* The first parameter is advertising set handle
				* The next two parameters are minimum and maximum advertising interval, both in
				* units of (milliseconds * 1.6).
				* The last two parameters are duration and maxevents left as default. */
				gecko_cmd_le_gap_set_advertise_timing(0, 1600, 1600, 0, 0);

				/* Start general advertising and enable connections. */
				gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);

#if 0
				gecko_cmd_hardware_set_soft_timer(32768/5, TIMER_ID_250MSEC, 0);
				gecko_cmd_hardware_set_soft_timer(32768, TIMER_ID_1SEC, 0);
#else
//				gecko_cmd_hardware_set_soft_timer(32768*60, TIMER_ID_60SEC, 0);
				gecko_cmd_hardware_set_soft_timer(32768, TIMER_ID_1SEC, 0);
#endif
				break;

			case gecko_evt_le_connection_opened_id:
				printLog("connection opened\r\n");
				/*Set timing parameters
				* Connection interval: 200 msec
				* Slave latency: as defined
				* Supervision timeout: 4500 msec The value in milliseconds must be larger than
				* (1 + latency) * max_interva * 2, where max_interval is given in milliseconds
				*/
				gecko_cmd_le_connection_set_timing_parameters(evt->data.evt_le_connection_opened.connection, 160, 160, 5, 450, 100, 0xFFFF);
		    	_conn_handle = evt->data.evt_le_connection_opened.connection;
		    	connected_with_mobile = 1;
				break;

			case gecko_evt_system_external_signal_id:
				if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_HALL_SENSOR)
				{
					printLog("Interrupt from HallSensor\r\n");
				}
				break;

			case gecko_evt_hardware_soft_timer_id:
				switch (evt->data.evt_hardware_soft_timer.handle)
				{
					case TIMER_ID_250MSEC:
//						read_sensor();
						break;

					case TIMER_ID_60SEC:

						read_VDD();

//						if(pMainCon->glucose.send_timer>0)	pMainCon->glucose.send_timer--;
						if(pMainCon->glucose.send_timer==0)
						{
							pMainCon->glucose.send_timer = pMainCon->glucose.max_timer;

							calc_glucose_sensor();

							memset(szBuff, 0, sizeof(szBuff));
							sprintf(szBuff, "G:%.4f  B:%.2f\r\n", pMainCon->glucose.calc_data, singleResult_VDD);

							len = strlen(szBuff);
							printLog("%s", szBuff);

							if(connected_with_mobile)
							{
								result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_peripheral_transmit, len, szBuff)->result;
							}
						}
						break;

					case TIMER_ID_1SEC:
						//SLG47004_op_amp_set();

						calc_glucose_sensor();							// 1초에 1ㅣ회 데이터 읽어오기...N Time 까지 평균을 내기로 함...

						pMainCon->glucose.calc_sum_data+=pMainCon->glucose.calc_data;
						pMainCon->glucose.calc_sum_count++;

						if(pMainCon->glucose.send_timer>0)	pMainCon->glucose.send_timer--;

						if(pMainCon->glucose.send_timer==0)
						{
							pMainCon->glucose.send_timer = pMainCon->glucose.max_timer;

							read_VDD();
//							read_Ref_Sensor();
//							calc_glucose_sensor();

//							pMainCon->glucose.calc_data = pMainCon->glucose.calc_sum_data/(double)pMainCon->glucose.calc_sum_count;

							memset(szBuff, 0, sizeof(szBuff));
//							sprintf(data, "R:%.4f   G:%.4f  B:%.2f\r\n", singleResult_Ref_Sensor, pMainCon->glucose.calc_data, singleResult_VDD);
							sprintf(szBuff, "G:%.4f  B:%.2f\r\n", pMainCon->glucose.calc_data, singleResult_VDD);

							pMainCon->glucose.calc_sum_data = 0;
							pMainCon->glucose.calc_sum_count= 0;


							len = strlen((char *)szBuff);
							printLog("%s", szBuff);

							if(connected_with_mobile)
							{
								result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_peripheral_transmit, len, szBuff)->result;
							}
						}


						break;

					default:
						break;
				}
			break;

			case gecko_evt_le_connection_closed_id:

				printLog("connection closed, reason: 0x%2.2x\r\n", evt->data.evt_le_connection_closed.reason);

				/* Check if need to boot to OTA DFU mode */
				if (boot_to_dfu)
				{
					/* Enter to OTA DFU mode */
					gecko_cmd_system_reset(2);
				}
				else
				{
					/* Restart advertising after client has disconnected */
					gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
				}
				break;

			/* Events related to OTA upgrading
			----------------------------------------------------------------------------- */

			/* Check if the user-type OTA Control Characteristic was written.
			* If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
			case gecko_evt_gatt_server_user_write_request_id:

				if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control)
				{
					/* Set flag to enter to OTA mode */
					boot_to_dfu = 1;
					/* Send response to Write Request */
					gecko_cmd_gatt_server_send_user_write_response(
									evt->data.evt_gatt_server_user_write_request.connection,
									gattdb_ota_control,
									bg_err_success);

					/* Close connection to enter to DFU OTA mode */
					gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
				}
				break;

			/* Add additional event handlers as your application requires */

			default:
				break;
		}
	}
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++) {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}

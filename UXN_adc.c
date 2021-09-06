/*
 * UXN_adc.c
 *
 *  Created on: 2020. 8. 4.
 *      Author: dooker
 */

#include "em_iadc.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "app.h"
#include "efr32bg22_iadc.h"

//#define _PCB_3ST_BOARD_

// Set CLK_ADC to 10MHz
#define CLK_SRC_ADC_FREQ          10000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ              10000000 // CLK_ADC - 10MHz max in normal mode


//#ifdef _PCB_3ST_BOARD_
#if 1
// When changing GPIO port/pins below, make sure to change xBUSALLOC macro's accordingly.
#define IADC_INPUT_0_BUS          	CDBUSALLOC
#define IADC_INPUT_0_BUSALLOC     	GPIO_CDBUSALLOC_CDEVEN0_ADC0
#define IADC_INPUT_1_BUS          	CDBUSALLOC
#define IADC_INPUT_1_BUSALLOC     	GPIO_CDBUSALLOC_CDODD0_ADC0

#define IADC_INPUT_BUS          	CDBUSALLOC
#define IADC_INPUT_BUSALLOC     	(GPIO_CDBUSALLOC_CDODD0_ADC0 | GPIO_CDBUSALLOC_CDEVEN0_ADC0)
#else
// When changing GPIO port/pins below, make sure to change xBUSALLOC macro's accordingly.
#define IADC_INPUT_0_BUS          ABUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_ABUSALLOC_AEVEN0_ADC0
#define IADC_INPUT_1_BUS          ABUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0

#define IADC_INPUT_BUS          ABUSALLOC
#define IADC_INPUT_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0
#endif

extern void delay_ms(uint32_t ms);

int32_t sample_VDD;
int32_t sample_Ref_Sensor;
int32_t sample_SENSOR;
double singleResult_VDD; // Volts
double singleResult_Ref_Sensor; // Volts
double singleResult_SENSOR; // Volts


void initIADC(void);
void read_adc(void);

static void IADC_disable(IADC_TypeDef *iadc)
{
#if defined(IADC_STATUS_SYNCBUSY)
	while ((iadc->STATUS & IADC_STATUS_SYNCBUSY) != 0U)
	{
		// Wait for synchronization to finish before disable
	}
#endif
	iadc->EN_CLR = IADC_EN_EN;
}

static void IADC_enable(IADC_TypeDef *iadc)
{
	iadc->EN_SET = IADC_EN_EN;
}
/**************************************************************************//**
 * @brief  Initialize IADC function
 *****************************************************************************/
void initIADC(void)
{
	// Declare init structs
	IADC_Init_t init = IADC_INIT_DEFAULT;
	IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
	IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
	IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

	// Enable IADC0 and GPIO clock branches
	CMU_ClockEnable(cmuClock_IADC0, true);

	// Reset IADC to reset configuration in case it has been modified by
	// other code
	IADC_reset(IADC0);

	// Select clock for IADC
	CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);  // FSRCO - 20MHz

	// Modify init structs and initialize
	init.warmup = iadcWarmupKeepWarm;

	// Set the HFSCLK prescale value here
	init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);					// 10000000 // CLK_SRC_ADC

	// Configuration 0 is used by both scan and single conversions by default
	// Use unbuffered AVDD as reference
	initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
	initAllConfigs.configs[0].analogGain= iadcCfgAnalogGain0P5x;


	// Divides CLK_SRC_ADC to set the CLK_ADC frequency
	initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
											 CLK_ADC_FREQ,
											 0,
											 iadcCfgModeNormal,
											 init.srcClkPrescale);

	// Assign pins to positive and negative inputs in differential mode
//	initSingleInput.posInput   = iadcPosInputPortAPin0;
	initSingleInput.posInput   = iadcPosInputAvdd;
	initSingleInput.negInput   = iadcNegInputGnd;

	// Initialize the IADC
	IADC_init(IADC0, &init, &initAllConfigs);

	// Initialize the Single conversion inputs
	IADC_initSingle(IADC0, &initSingle, &initSingleInput);

	// Allocate the analog bus for ADC0 inputs
	GPIO->IADC_INPUT_BUS |= IADC_INPUT_BUSALLOC;

	IADC_disable(IADC0);
}

/**************************************************************************//**
 * @brief  Read ADC data
 *****************************************************************************/
void read_VDD(void)
{
	IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
	IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

	// Assign pins to positive and negative inputs in differential mode
	initSingleInput.posInput   = iadcPosInputAvdd;
	initSingleInput.negInput   = iadcNegInputGnd;
	IADC_initSingle(IADC0, &initSingle, &initSingleInput);

	IADC_enable(IADC0);

	// Start IADC conversion
	IADC_command(IADC0, iadcCmdStartSingle);

	// Wait for conversion to be complete
	while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
				| _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

	// Get ADC result
	sample_VDD = IADC_pullSingleFifoResult(IADC0).data;

	// Calculate input voltage:
	// For single-ended the result range is 0 to +Vref, i.e., 12 bits for the conversion value.
	// The result of AVDD measurement is attenuated by a factor of 4
	// Therefore in order to get the full ADD measurement, the result needs to be multiplied by 4
	singleResult_VDD = (sample_VDD * 1.2 * 4) / 0xFFF;

//	printLog("VDD Value = %f[V]\r\n", singleResult_VDD);

	IADC_disable(IADC0);
}


/**************************************************************************//**
 * @brief  Read ADC data
 *****************************************************************************/
void read_Work_Sensor(void)
{
	IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
	IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

	// Assign pins to positive and negative inputs in differential mode
	initSingleInput.posInput   = iadcPosInputPortCPin2;
	initSingleInput.negInput   = iadcNegInputGnd;
	IADC_initSingle(IADC0, &initSingle, &initSingleInput);

	IADC_enable(IADC0);

	// Start IADC conversion
	IADC_command(IADC0, iadcCmdStartSingle);

	// Wait for conversion to be complete
	while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
				| _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

	// Get ADC result
	sample_Ref_Sensor = IADC_pullSingleFifoResult(IADC0).data;

	// Calculate input voltage:
	// For single-ended the result range is 0 to +Vref, i.e., 12 bits for the conversion value.
	// The result of AVDD measurement is attenuated by a factor of 4
	// Therefore in order to get the full ADD measurement, the result needs to be multiplied by 4
	singleResult_Ref_Sensor = (sample_Ref_Sensor * 1.2) / 0xFFF;

//	printLog("VDD Value = %f[V]\r\n", singleResult_Ref_Sensor);

	IADC_disable(IADC0);
}

/**************************************************************************//**
 * @brief  Read ADC data
 *****************************************************************************/
void read_Ref_Sensor(void)
{
	IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
	IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

	// Assign pins to positive and negative inputs in differential mode
	initSingleInput.posInput   = iadcPosInputPortCPin3;
	initSingleInput.negInput   = iadcNegInputGnd;
	IADC_initSingle(IADC0, &initSingle, &initSingleInput);

	IADC_enable(IADC0);

	// Start IADC conversion
	IADC_command(IADC0, iadcCmdStartSingle);

	// Wait for conversion to be complete
	while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
				| _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

	// Get ADC result
	sample_Ref_Sensor = IADC_pullSingleFifoResult(IADC0).data;

	// Calculate input voltage:
	// For single-ended the result range is 0 to +Vref, i.e., 12 bits for the conversion value.
	// The result of AVDD measurement is attenuated by a factor of 4
	// Therefore in order to get the full ADD measurement, the result needs to be multiplied by 4
	singleResult_Ref_Sensor = (sample_Ref_Sensor * 1.2) / 0xFFF;

//	printLog("VDD Value = %f[V]\r\n", singleResult_Ref_Sensor);

	IADC_disable(IADC0);
}


double read_sensor(void)
{
	IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
	IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;
//	static unsigned int count_read = 0;

	// Assign pins to positive and negative inputs in differential mode
#ifdef _PCB_3ST_BOARD_				// OLD
	initSingleInput.posInput   = iadcPosInputPortCPin4;
#else
	initSingleInput.posInput   = iadcPosInputPortCPin1;
#endif
	initSingleInput.negInput   = iadcNegInputGnd;

	IADC_initSingle(IADC0, &initSingle, &initSingleInput);

	IADC_enable(IADC0);

	// Start IADC conversion
	IADC_command(IADC0, iadcCmdStartSingle);

	// Wait for conversion to be complete
	while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
				| _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

	// Get ADC result
	sample_SENSOR = IADC_pullSingleFifoResult(IADC0).data;

	// Calculate input voltage:
	// For single-ended the result range is 0 to +Vref, i.e., 12 bits for the conversion value.
	// The result of AVDD measurement is attenuated by a factor of 4
	// Therefore in order to get the full ADD measurement, the result needs to be multiplied by 4
	singleResult_SENSOR = sample_SENSOR * 1.2 / 0xFFF;

	IADC_disable(IADC0);

	return singleResult_SENSOR;

#if 0

#ifdef _SODYKIM_CODE_
	int i;
	int max_ps=0;
	int min_ps=0;
	double max = 0;
	double min = 0xFFFFFFFF;
	double sum = 0;
#endif

	IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
	IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;
//	static unsigned int count_read = 0;

	// Assign pins to positive and negative inputs in differential mode
	initSingleInput.posInput   = iadcPosInputPortCPin4;
	initSingleInput.negInput   = iadcNegInputGnd;
	IADC_initSingle(IADC0, &initSingle, &initSingleInput);

	IADC_enable(IADC0);

	// Start IADC conversion
	IADC_command(IADC0, iadcCmdStartSingle);

	// Wait for conversion to be complete
	while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
				| _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

	// Get ADC result
	sample_SENSOR = IADC_pullSingleFifoResult(IADC0).data;

	// Calculate input voltage:
	// For single-ended the result range is 0 to +Vref, i.e., 12 bits for the conversion value.
	// The result of AVDD measurement is attenuated by a factor of 4
	// Therefore in order to get the full ADD measurement, the result needs to be multiplied by 4
	singleResult_SENSOR = sample_SENSOR * 1.2 / 0xFFF;

#if 0
//#ifdef _SODYKIM_CODE_
	{

		pMainCon->glucose.pick_buff[pMainCon->glucose.pick_count++] = singleResult_SENSOR;

		if(pMainCon->glucose.pick_count>=MAX_PICK_SIZE)											// 250ms per avoid 2 adc value...max & min  -> 2 adc value sum / 2
		{
			pMainCon->glucose.pick_count = 0;

			for(i=0;i<MAX_PICK_SIZE;i++)
			{
				if(pMainCon->glucose.pick_buff[i] > max)
				{
					max = pMainCon->glucose.pick_buff[i];
					max_ps = i;
				}

				if(pMainCon->glucose.pick_buff[i] < min)
				{
					min = pMainCon->glucose.pick_buff[i];
					min_ps = i;
				}
			}
			sum = 0;
			for(i=0;i<MAX_PICK_SIZE;i++)
			{
				if((i!=max_ps) && (i!=min_ps))
				{
					sum+=pMainCon->glucose.pick_buff[i];
				}
			}

			singleResult_SENSOR = sum/2.;

//---------------------------------------------------------------------------------------------------
//	1sec buffering
//---------------------------------------------------------------------------------------------------
			pMainCon->glucose.buffer[pMainCon->glucose.sum_count++] = singleResult_SENSOR;

			if(pMainCon->glucose.sum_count >= MAX_GLUCOSE_SIZE)
			{
				pMainCon->glucose.sum_count = 0;

				pMainCon->glucose.sum_flag = ENABLE;
			}

			sum = 0;
			if(pMainCon->glucose.sum_flag == ENABLE)						// 60개 이상 다 차면...그 뒤 부터는 평균으로 처리 한다....
			{
				for(i=0;i<MAX_GLUCOSE_SIZE;i++)
				{
					sum+=pMainCon->glucose.buffer[i];
				}

				singleResult_SENSOR = sum/MAX_GLUCOSE_SIZE;
			}
		}
	}
#endif



//	if(count_read%5 == 0)
//		printLog("[%04ld]SENSOR Value = %f[V]\r\n", count_read, singleResult_SENSOR);

//	if(count_read++ == 9999)
//		count_read = 0;

	IADC_disable(IADC0);

	return singleResult_SENSOR;
#endif
}

#define CALC_MAX_GLUCOSE_DATA	25
void calc_glucose_sensor(void)
{
	int i;
	double sum_glucose=0;

#if 1
	for(i=0;i<CALC_MAX_GLUCOSE_DATA;i++)
	{
		sum_glucose+=read_sensor();
		delay_ms(2);
	}

	pMainCon->glucose.calc_data = sum_glucose/CALC_MAX_GLUCOSE_DATA;
#else
	pMainCon->glucose.calc_data = read_sensor();
#endif

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	GPIO INITIAL
//
void init_GPIO(void)
{
#ifdef _PCB_3ST_BOARD_
	GPIO_PinModeSet(gpioPortA, 3, gpioModePushPull, 1);					// POWER ENABLE HIGH
	GPIO_PinModeSet(gpioPortA, 4, gpioModePushPull, 1);					// RP517		HIGH
#else

#if 0
	//	GPIO_PinModeSet(gpioPortA, 3, gpioModePushPull, 1);					// POWER ENABLE HIGH
	GPIO_PinModeSet(gpioPortC, 4, gpioModePushPull, 1);					// RP517		HIGH
	GPIO_PinModeSet(gpioPortA, 4, gpioModeInput, 0);					// PORTA4 INPUT
#else
	GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 1);					// MCP1810_PWR_EN		HIGH

	GPIO_PinModeSet(gpioPortC, 0, gpioModeInput, 0);					// ADC
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);					// WORK ADC
	GPIO_PinModeSet(gpioPortC, 2, gpioModeInput, 0);					// REF ADC

#endif

#endif
}

void POWER_Enable(unsigned int out)
{
#ifdef _PCB_3ST_BOARD_
	if(out)
		GPIO_PinOutSet(gpioPortC, 4);
	else
		GPIO_PinOutClear(gpioPortC, 4);
#else
	if(out)
		GPIO_PinOutSet(gpioPortA, 6);
	else
		GPIO_PinOutClear(gpioPortA, 6);
#endif
}





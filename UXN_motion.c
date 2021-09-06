/*
 * UXN_i2c.c
 *
 *  Created on: 2020. 8. 6.
 *      Author: dooker
 */

#include <stdio.h>
#include "em_cmu.h"
#include "i2cspm.h"
#include "lis2dh12_reg.h"
#include "gpiointerrupt.h"

#include "SLG47004.h"

#include "app.h"

//#define I2C_ADDR_MOTION_WRITE		0x30
//#define I2C_ADDR_MOTION_READ		0x31
#define I2C_ADDR_MOTION_WRITE		0x32
#define I2C_ADDR_MOTION_READ		0x33

#define I2C_ADDR_SLG47004_WRITE		0x08
#define I2C_ADDR_SLG47004_READ		0x09


lis2dh12_ctx_t 	dev_ctx;
slg47004_ctx_t	op_amp_ctx;

I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
uint8_t i2c_read_data[32];
uint8_t i2c_write_data[2];

static uint8_t whoamI;

int32_t I2C_WRITE(uint8_t addr, uint8_t reg_addr, uint8_t command)
{
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;

	I2C_TypeDef *i2c;
	i2c = i2cInit.port;

	seq.addr  = addr;
	seq.flags = I2C_FLAG_WRITE;
	/* Select command to issue */
	i2c_write_data[0] = reg_addr;
	i2c_write_data[1] = command;
	seq.buf[0].data   = i2c_write_data;
	seq.buf[0].len    = 2;

	/* Select location/length of data to be read */
	//  seq.buf[1].data = i2c_read_data;
	//  seq.buf[1].len  = 0;

	ret = I2CSPM_Transfer(i2c, &seq);

	if (ret != i2cTransferDone)
	{
		return((int) ret);
	}

	return((int) 0);
}

uint8_t I2C_READ(uint8_t addr, uint8_t reg_addr, uint8_t read_len)
{
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;

	I2C_TypeDef *i2c;
	i2c = i2cInit.port;

	memset(i2c_read_data, 0, sizeof(i2c_read_data));

	seq.addr  = addr;
	seq.flags = I2C_FLAG_WRITE_READ;

	/* Select location/length of data to be write */
	seq.buf[0].data   = &reg_addr;
	seq.buf[0].len    = 1;

	/* Select location/length of data to be read */
	seq.buf[1].data = i2c_read_data;
	seq.buf[1].len  = read_len;

	ret = I2CSPM_Transfer(i2c, &seq);

	if (ret != i2cTransferDone)
	{
		return((uint8_t) 0);
	}

	return 1;
}





/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	/* Write multiple command */
//	reg |= 0x80;
	I2C_WRITE(I2C_ADDR_MOTION_WRITE, reg, bufp[0]);

	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	uint8_t i;

	/* Read multiple command */
	if(len > 1)
		reg |= 0x80;

	I2C_READ(I2C_ADDR_MOTION_READ, reg, len);

	for(i=0; i<len; i++)
		bufp[i] = i2c_read_data[i];

	return 0;
}

void sensor_init_Motion(void)
{
	uint16_t	retry_count = 1;

	I2CSPM_Init(&i2cInit);

	do {
		// Check device ID
		lis2dh12_device_id_get(&dev_ctx, &whoamI);

//		I2C_READ(0xAC, uint8_t reg_addr, uint8_t read_len)

//		printLog("LIS2DH12: READ ID is 0x%x\r\n", whoamI);

		if(retry_count > 10)
		{
//			printLog("Motion : Error to identify LIS2DH12(0x%x)\r\n", whoamI);
			return;
		}

		retry_count++;
	} while(whoamI != LIS2DH12_ID);

//	printLog("Motion : Success to identify LIS2DH12 by %d times(0x%x)\r\n", retry_count, whoamI);


#if 1	// continuous reading sensor value
	// Enable Block Data Update
	lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	// Set Output Data Rate to 1Hz
	lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_1Hz);

	// Set full scale to 2g
	lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);

	// Enable temperature sensor
//	lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_ENABLE);

	//  Set device in continuous mode with 12 bit resol.
//	lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);

	//  Set device in low power mode with 8 bit resol.
	lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_LP_8bit);
#else	// Generate Interrupt

	// For INT1
	uint8_t ctrl_reg3_val = 0x40;
	uint8_t ctrl_reg4_val = 0x00;

	uint8_t ctrl_int1_cfg_val = 0x0a;
	uint8_t ctrl_int1_ths_val = 0x14;
	uint8_t ctrl_int1_duration_val = 0x00;

	// For INT2
	uint8_t ctrl_reg6_val = 0x40;

	uint8_t ctrl_int2_cfg_val = 0x0a;
	uint8_t ctrl_int2_ths_val = 0x14;
	uint8_t ctrl_int2_duration_val = 0x00;

	/* Initialization of sensor */
	lis2dh12_pin_int1_config_set(&dev_ctx, &ctrl_reg3_val);	 			/* CTRL_REG3(22h): IA1 interrupt on INT1 pin */
	lis2dh12_pin_int2_config_set(&dev_ctx, &ctrl_reg6_val);	 			/* CTRL_REG6(25h): IA2 interrupt on INT2 pin */
	lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);			 			/* CTRL_REG4(23h): Set Full-scale to +/-2g */

	// Latch Interrupt pin notification
//	lis2dh12_int1_pin_notification_mode_set(&dev_ctx, LIS2DH12_INT1_LATCHED);
//	lis2dh12_int2_pin_notification_mode_set(&dev_ctx, LIS2DH12_INT2_LATCHED);

	/* Wakeup recognition enable */
	lis2dh12_int1_gen_conf_set(&dev_ctx, &ctrl_int1_cfg_val);  			/* INT1_CFG(30h): INT1 Configuration */
	lis2dh12_int1_gen_threshold_set(&dev_ctx, ctrl_int1_ths_val);		/* INT1_THS(32h): INT1 Threshold set */
	lis2dh12_int1_gen_duration_set(&dev_ctx, ctrl_int1_duration_val);	/* INT1_DURATION(33h): INT1 Duration set */

	/* Wakeup recognition enable */
	lis2dh12_int2_gen_conf_set(&dev_ctx, &ctrl_int2_cfg_val);  			/* INT1_CFG(34h): INT1 Configuration */
	lis2dh12_int2_gen_threshold_set(&dev_ctx, ctrl_int2_ths_val);		/* INT1_THS(36h): INT1 Threshold set */
	lis2dh12_int2_gen_duration_set(&dev_ctx, ctrl_int2_duration_val);	/* INT1_DURATION(37h): INT1 Duration set */

	/* Start sensor */
//	lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_100Hz); 				/* CTRL_REG1(20h): Start sensor at ODR 100Hz Low-power mode */
	lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_10Hz); 				/* CTRL_REG1(20h): Start sensor at ODR 100Hz Low-power mode */
#endif
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	SLG74004
//

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
static int32_t slg47004_write_user_reg(I2C_TypeDef *i2c, uint8_t addr, int8_t command, int8_t data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  uint8_t                    i2c_write_data[2];

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = command;
  i2c_write_data[1] = data;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;

  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone) {
    return((int) ret);
  }

  return((int) 0);
}



static int32_t slg47004_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	/* Write multiple command */
//	reg |= 0x80;
	I2C_WRITE(I2C_ADDR_SLG47004_WRITE, reg, bufp[0]);

	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t slg47004_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	uint8_t i;

	/* Read multiple command */
	if(len > 1)
		reg |= 0x80;

	I2C_READ(I2C_ADDR_SLG47004_READ, reg, len);

	for(i=0; i<len; i++)
		bufp[i] = i2c_read_data[i];

	return 0;
}



void SLG47004_op_amp_set(void)
{
//	slg47004_op_amp_acmp_set(&op_amp_ctx ,0xC0, 0xC1);
	slg47004_write_user_reg(I2C0, I2C_ADDR_SLG47004_WRITE, 0x5F, 0xC0);
	slg47004_write_user_reg(I2C0, I2C_ADDR_SLG47004_WRITE, 0x60, 0xC1);

}

void SLG47004_Init(void)
{
	I2CSPM_Init(&i2cInit);

	slg47004_write_user_reg(I2C0, I2C_ADDR_SLG47004_WRITE, 0x5F, 0xC0);
	slg47004_write_user_reg(I2C0, I2C_ADDR_SLG47004_WRITE, 0x60, 0xC1);
}


void init_motion(void)
{
	/***************************************/
	// Initialize mems driver interface
	/***************************************/

//	dev_ctx.write_reg = platform_write;
//	dev_ctx.read_reg = platform_read;

//	sensor_init_Motion();

#if 1
	op_amp_ctx.write_reg = slg47004_write;
	op_amp_ctx.read_reg  = slg47004_read;

	SLG47004_Init();
#endif
}

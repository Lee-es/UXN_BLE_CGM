/*
 ******************************************************************************
 * @file    M24SR.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          lis2dh12_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef M24SR_H
#define M24SR_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>




/**
  * @}
  *
  */

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	M24SR_DRIVER
//
/** @addtogroup M24SR_Driver
 * @{
 */

/** @addtogroup drv_M24SR
 * @{
 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  APDU-Header command structure
 */
typedef struct {
	uint8_t CLA; /* Command class */
	uint8_t INS; /* Operation code */
	uint8_t P1; /* Selection Mode */
	uint8_t P2; /* Selection Option */
} C_APDU_Header;

/**
 * @brief  APDU-Body command structure
 */
typedef struct {
	uint8_t LC; /* Data field length */
	const uint8_t *pData; /* Command parameters */
	uint8_t LE; /* Expected length of data to be returned */
} C_APDU_Body;

/**
 * @brief  APDU Command structure
 */
typedef struct {
	C_APDU_Header Header;
	C_APDU_Body Body;
} C_APDU;

/**
 * @brief  SC response structure
 */
typedef struct {
	uint8_t *pData; /* Data returned from the card */ // pointer on the transceiver buffer = ReaderRecBuf[CR95HF_DATA_OFFSET ];
	uint8_t SW1; /* Command Processing status */
	uint8_t SW2; /* Command Processing qualification */
} R_APDU;

/**
 * @brief  GPO mode structure
 */
typedef enum {
	RF_GPO = 0, I2C_GPO = 1
} M24SR_GPO_MODE;

typedef enum {
	M24SR_WAITINGTIME_POLLING,
	M24SR_INTERRUPT_GPO
} M24SR_WAITINGTIME_MGMT;

/* Exported constants --------------------------------------------------------*/

/** @defgroup lib_M24SR_Exported_Constants
 * @{
 */

/* ---------------------- status code ----------------------------------------*/
#define UB_STATUS_OFFSET			4
#define LB_STATUS_OFFSET  			3
#define I_AM_M24SR                	((uint8_t)0xB4)
#define I_AM_M24SR_AUTOMOTIVE       ((uint8_t)0xBC)

#define M24SR_NBBYTE_INVALID		0xFFFE

/** @defgroup drv_M24SR_File_Identifier
 * @{
 */
#define SYSTEM_FILE_ID_BYTES 	{0xE1,0x01}
#define CC_FILE_ID_BYTES 		{0xE1,0x03}
#define NDEF_FILE_ID			0x0001
/**
 * @}
 */

/** @defgroup drv_M24SR_Password_Management
 * @{
 */
#define READ_PWD								0x0001
#define WRITE_PWD								0x0002
#define I2C_PWD									0x0003

/*-------------------------- Verify command answer ----------------------------*/
/**
 * @}
 */

/** @defgroup drv_M24SR_Command_Management
 * @{
 */

/* special M24SR command ----------------------------------------------------------------------*/
#define M24SR_OPENSESSION_COMMAND 	{0x26}
#define M24SR_KILLSESSION_COMMAND 	{0x52}

/* APDU Command: class list -------------------------------------------*/
#define C_APDU_CLA_DEFAULT	  	0x00
#define C_APDU_CLA_ST			0xA2

/*------------------------ Data Area Management Commands ---------------------*/
#define C_APDU_SELECT_FILE     0xA4
#define C_APDU_GET_RESPONCE    0xC0
#define C_APDU_STATUS          0xF2
#define C_APDU_UPDATE_BINARY   0xD6
#define C_APDU_READ_BINARY     0xB0
#define C_APDU_WRITE_BINARY    0xD0
#define C_APDU_UPDATE_RECORD   0xDC
#define C_APDU_READ_RECORD     0xB2

/*-------------------------- Safety Management Commands ----------------------*/
#define C_APDU_VERIFY          0x20
#define C_APDU_CHANGE          0x24
#define C_APDU_DISABLE         0x26
#define C_APDU_ENABLE          0x28

/*-------------------------- Gpio Management Commands ------------------------*/
#define C_APDU_INTERRUPT       0xD6

/*  Length	----------------------------------------------------------------------------------*/
#define M24SR_STATUS_NBBYTE							2
#define M24SR_CRC_NBBYTE							2
#define M24SR_STATUSRESPONSE_NBBYTE					5
#define M24SR_DESELECTREQUEST_COMMAND			{0xC2,0xE0,0xB4}
#define M24SR_DESELECTRESPONSE_NBBYTE				3
#define M24SR_WATINGTIMEEXTRESPONSE_NBBYTE			4
#define M24SR_PASSWORD_NBBYTE						0x10
#define M24SR_SELECTAPPLICATION_COMMAND	{0xD2,0x76,0x00,0x00,0x85,0x01,0x01}
/*  Command structure	------------------------------------------------------------------------*/
#define M24SR_CMDSTRUCT_SELECTAPPLICATION			0x01FF
#define M24SR_CMDSTRUCT_SELECTCCFILE				0x017F
#define M24SR_CMDSTRUCT_SELECTNDEFFILE				0x017F
#define M24SR_CMDSTRUCT_READBINARY					0x019F
#define M24SR_CMDSTRUCT_UPDATEBINARY				0x017F
#define M24SR_CMDSTRUCT_VERIFYBINARYWOPWD			0x013F
#define M24SR_CMDSTRUCT_VERIFYBINARYWITHPWD			0x017F
#define M24SR_CMDSTRUCT_CHANGEREFDATA				0x017F
#define M24SR_CMDSTRUCT_ENABLEVERIFREQ				0x011F
#define M24SR_CMDSTRUCT_DISABLEVERIFREQ				0x011F
#define M24SR_CMDSTRUCT_SENDINTERRUPT				0x013F
#define M24SR_CMDSTRUCT_GPOSTATE					0x017F

/*  Command structure Mask -------------------------------------------------------------------*/
#define M24SR_PCB_NEEDED				0x0001		/* PCB byte present or not */
#define M24SR_CLA_NEEDED				0x0002 		/* CLA byte present or not */
#define M24SR_INS_NEEDED				0x0004 		/* Operation code present or not*/
#define M24SR_P1_NEEDED					0x0008		/* Selection Mode  present or not*/
#define M24SR_P2_NEEDED					0x0010		/* Selection Option present or not*/
#define M24SR_LC_NEEDED					0x0020		/* Data field length byte present or not */
#define M24SR_DATA_NEEDED				0x0040		/* Data present or not */
#define M24SR_LE_NEEDED					0x0080		/* Expected length present or not */
#define M24SR_CRC_NEEDED				0x0100		/* 2 CRC bytes present	or not */

#define M24SR_DID_NEEDED				0x08			/* DID byte present or not */

/**
 * @}
 */

/*  Offset	----------------------------------------------------------------------------------*/
#define M24SR_OFFSET_PCB													0
#define M24SR_OFFSET_CLASS													1
#define M24SR_OFFSET_INS													2
#define M24SR_OFFSET_P1														3

/*  mask	------------------------------------------------------------------------------------*/
#define M24SR_MASK_BLOCK													0xC0
#define M24SR_MASK_IBLOCK													0x00
#define M24SR_MASK_RBLOCK													0x80
#define M24SR_MASK_SBLOCK													0xC0


#define GETMSB(val) 		( (uint8_t) ((val & 0xFF00 )>>8) )

/** @brief Get Least Significant Byte
 * @param  val: number where LSB must be extracted
 * @retval LSB
 */
#define GETLSB(val) 		( (uint8_t) (val & 0x00FF ))

/** @brief Used to toggle the block number by adding 0 or 1 to default block number value
 * @param  val: number to know if incrementation is needed
 * @retval  0 or 1 if incrementation needed
 */
#define TOGGLE(val) 		((val != 0x00)? 0x00 : 0x01)

typedef struct {
	C_APDU command;
	//static R_APDU 						Response;
	uint8_t dataBuffer[0xFF];
	uint8_t uM24SRbuffer[0xFF];
	uint8_t uDIDbyte;
} M24SR_DrvDataTypeDef;

typedef void* M24SR_InitTypeDef;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	M24SR
//
#define	    NFC_SUCCESS  0

typedef enum {
	M24SR_SUCCESS=NFC_SUCCESS,
	M24SR_ERROR=0x6F00,
	M24SR_FILE_OVERFLOW_LE = 0x6280,
	M24SR_EOF = 0x6282,
	M24SR_PASSWORD_REQUIRED = 0x6300,
	M24SR_PASSWORD_INCORRECT = 0x63C0,
	M24SR_PASSWORD_INCORRECT1RETRY = 0x63C1,
	M24SR_PASSWORD_INCORRECT2RETRY = 0x63C2,
	M24SR_WRONG_LENGHT = 0x6700,
	M24SR_UNSUCESSFUL_UPDATING = 0x6581,
	M24SR_INCOPATIBLE_COMMAND= 0x6981,
	M24SR_SECURITY_UNSATISFIED = 0x6982,
	M24SR_REFERENCE_DATA_NOT_USABLE = 0x6984,

	M24SR_INCORRECT_PARAMETER = 0x6a80,
	M24SR_FILE_NOT_FOUND=0x6a82,
	M24SR_FILE_OVERFLOW_LC = 0x6A84, //TODO difference with Le??

	M24SR_INCORRECT_P1_OR_P2 = 0x6A86, //TODO better name?
	M24SR_RF_SESSION_KILLED=0x6500,
	M24SR_INS_NOT_SUPPORTED=0x6D00,
	M24SR_CLASS_NOT_SUPPORTED=0x6E00,

	//IOError
	M24SR_IO_ERROR_I2CTIMEOUT=0x0011,
	M24SR_IO_ERROR_CRC=0x0012,
	M24SR_IO_ERROR_NACK=0x0013,
	M24SR_IO_ERROR_PARAMETER=0x0014,
	M24SR_IO_ERROR_NBATEMPT=0x0015,
	M24SR_IO_NOACKNOWLEDGE=0x0016,
	M24SR_IO_PIN_NOT_CONNECTED=0x0017
} StatusTypeDef;

/**
 * @brief  GPO state structure
 */
typedef enum {
	HIGH_IMPEDANCE = 0,
	SESSION_OPENED =1,
	WIP=2,
	I2C_ANSWER_READY=3,
	INTERRUPT=4,
	STATE_CONTROL=5
} NFC_GPO_MGMT;

/**
 * Possible password to set.
 */
typedef enum{
	ReadPwd,   //!< Password to use before reading the tag
	WritePwd,  //!< Password to use before writing the tag
	I2CPwd,    //!< Root password, used only through nfc
}PasswordType_t;

/**
 * Default password used to change the write/read permission
 */
//static const uint8_t DEFAULT_PASSWORD[16];

/**
 * Default gpo status -> the gpo will remain high
 */
static const NFC_GPO_MGMT DEFAULT_GPO_STATUS=HIGH_IMPEDANCE;




/**
 * Command that the component can accept
 */
typedef enum{
	NONE,                            //!< NONE
	DESELECT,                        //!< DESELECT
	SELECT_APPLICATION,              //!< SELECT_APPLICATION
	SELECT_CC_FILE,                  //!< SELECT_CC_FILE
	SELECT_NDEF_FILE,                //!< SELECT_NDEF_FILE
	SELECT_SYSTEM_FILE,              //!< SELECT_SYSTEM_FILE
	READ,                            //!< READ
	UPDATE,                          //!< UPDATE
	VERIFY,                          //!< VERIFY
	MANAGE_I2C_GPO,                  //!< MANAGE_I2C_GPO
	MANAGE_RF_GPO,                   //!< MANAGE_RF_GPO
	CHANGE_REFERENCE_DATA,           //!< CHANGE_REFERENCE_DATA
	ENABLE_VERIFICATION_REQUIREMENT, //!< ENABLE_VERIFICATION_REQUIREMENT
	DISABLE_VERIFICATION_REQUIREMENT,//!< DISABLE_VERIFICATION_REQUIREMENT
	ENABLE_PERMANET_STATE,           //!< ENABLE_PERMANET_STATE
	DISABLE_PERMANET_STATE,          //!< DISABLE_PERMANET_STATE
}M24SR_command_t;


/**
	 * User parameter used to invoke a command,
	 * it is used to provide the data back with the response
 */
typedef struct{
	uint8_t *data; //!< data
	uint16_t length; //!< number of bytes in the data array
	uint16_t offset; //!< offset parameter used in the read/write command
}M24SR_command_data_t;

/**
 * Communication mode used by this device
 */
typedef enum{
	SYNC,//!< SYNC wait the command response before returning
	ASYNC//!< ASYNC use a callback to notify the end of a command
}M24SR_communication_t;

/**
 * Type of communication being used
 */
M24SR_communication_t mCommunicationType;

/**
 * Last pending command
 */
M24SR_command_t mLastCommandSend;

/**
 * Parameter used to invoke the last command
 */
M24SR_command_data_t mLastCommandData;



#ifdef __cplusplus
}
#endif

#endif /* M24SR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

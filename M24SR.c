/*
 ******************************************************************************
 * @file    M24SR.c
 * @author  Sensors Software Solution Team
 * @brief   LIS2DH12 driver file
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

#include <string.h>
#include "M24SR.h"



#define MAX_OPERATION_SIZE         246
#define MAX_PAYLOAD                241

/** value returned by the NFC chip when a command is successfully completed */
const uint16_t NFC_COMMAND_SUCCESS = 0x9000;
/** I2C nfc address */
#define M24SR_ADDR                 0xAC

#define SYSTEM_FILE_ID_BYTES      {0xE1,0x01}
#define CC_FILE_ID_BYTES          {0xE1,0x03}

#define UB_STATUS_OFFSET           4
#define LB_STATUS_OFFSET           3

/* APDU command: class list */
#define C_APDU_CLA_DEFAULT         0x00
#define C_APDU_CLA_ST              0xA2

/* data area management commands */
#define C_APDU_SELECT_FILE         0xA4
#define C_APDU_GET_RESPONCE        0xC0
#define C_APDU_STATUS              0xF2
#define C_APDU_UPDATE_BINARY       0xD6
#define C_APDU_READ_BINARY         0xB0
#define C_APDU_WRITE_BINARY        0xD0
#define C_APDU_UPDATE_RECORD       0xDC
#define C_APDU_READ_RECORD         0xB2

/* safety management commands */
#define C_APDU_VERIFY              0x20
#define C_APDU_CHANGE              0x24
#define C_APDU_DISABLE             0x26
#define C_APDU_ENABLE              0x28

/* GPO management commands */
#define C_APDU_INTERRUPT           0xD6

/* length */
#define STATUS_LENGTH                    2
#define CRC_LENGTH                       2
#define STATUS_RESPONSE_LENGTH           5
#define DESELECT_RESPONSE_LENGTH         3
#define WATING_TIME_EXT_RESPONSE_LENGTH  4
#define PASSWORD_LENGTH                  16

#define DESELECT_REQUEST_COMMAND     {0xC2,0xE0,0xB4}
#define SELECT_APPLICATION_COMMAND   {0xD2,0x76,0x00,0x00,0x85,0x01,0x01}

/* command structure mask */
#define CMD_MASK_SELECT_APPLICATION     0x01FF
#define CMD_MASK_SELECT_CC_FILE         0x017F
#define CMD_MASK_SELECT_NDEF_FILE       0x017F
#define CMD_MASK_READ_BINARY            0x019F
#define CMD_MASK_UPDATE_BINARY          0x017F
#define CMD_MASK_VERIFY_BINARY_WO_PWD   0x013F
#define CMD_MASK_VERIFY_BINARY_WITH_PWD 0x017F
#define CMD_MASK_CHANGE_REF_DATA        0x017F
#define CMD_MASK_ENABLE_VERIFREQ        0x011F
#define CMD_MASK_DISABLE_VERIFREQ       0x011F
#define CMD_MASK_SEND_INTERRUPT         0x013F
#define CMD_MASK_GPO_STATE              0x017F

/* command structure values for the mask */
#define PCB_NEEDED                0x0001      /* PCB byte present or not */
#define CLA_NEEDED                0x0002      /* CLA byte present or not */
#define INS_NEEDED                0x0004      /* Operation code present or not*/
#define P1_NEEDED                 0x0008      /* Selection Mode  present or not*/
#define P2_NEEDED                 0x0010      /* Selection Option present or not*/
#define LC_NEEDED                 0x0020      /* Data field length byte present or not */
#define DATA_NEEDED               0x0040      /* Data present or not */
#define LE_NEEDED                 0x0080      /* Expected length present or not */
#define CRC_NEEDED                0x0100      /* 2 CRC bytes present  or not */
#define DID_NEEDED                0x08        /* DID byte present or not */

/*  offset */
#define OFFSET_PCB                0
#define OFFSET_CLASS              1
#define OFFSET_INS                2
#define OFFSET_P1                 3

/*  mask */
#define MASK_BLOCK                0xC0
#define MASK_I_BLOCK              0x00
#define MASK_R_BLOCK              0x80
#define MASK_S_BLOCK              0xC0

#define GETMSB(val)               ((uint8_t) ((val & 0xFF00)>>8))
#define GETLSB(val)               ((uint8_t) (val & 0x00FF))


/** default password, also used to enable super user mode through the I2C channel */
const uint8_t default_password[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

extern uint8_t i2c_read_data[32];
extern uint8_t i2c_write_data[2];


extern int32_t I2C_WRITE(uint8_t addr, uint8_t reg_addr, uint8_t command);
extern uint8_t I2C_READ(uint8_t addr, uint8_t reg_addr, uint8_t read_len);


////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	I2C INITIAL & READ & WRITE
//


/**
 * @brief This function updates the CRC
 */
static uint16_t update_crc(uint8_t ch, uint16_t *lpw_crc) {
    ch = (ch ^ (uint8_t) ((*lpw_crc) & 0x00FF));
    ch = (ch ^ (ch << 4));
    *lpw_crc = (*lpw_crc >> 8) ^ ((uint16_t) ch << 8) ^ ((uint16_t) ch << 3) ^ ((uint16_t) ch >> 4);

    return (*lpw_crc);
}













/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

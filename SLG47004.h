

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SLG47004_H
#define SLG47004_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup SLG47004
  * @{
  *
  */

/** @defgroup SLG47004_sensors_common_types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

/**
  * @defgroup axisXbitXX_t
  * @brief    These unions are useful to represent different sensors data type.
  *           These unions are not need by the driver.
  *
  *           REMOVING the unions you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */


#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

#endif /* MEMS_SHARED_TYPES */

/**
  * @}
  *
  */

/** @addtogroup LIS3MDL_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*slg47004_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*slg47004_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  slg47004_write_ptr  write_reg;
  slg47004_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} slg47004_ctx_t;


#define ACMP_Vref0_ADDR	0x5F
#define ACMP_Vref1_ADDR	0x60

#endif /* SLG47004_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

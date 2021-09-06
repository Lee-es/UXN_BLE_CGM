

#include <string.h>
#include "SLG47004.h"

int32_t slg47004_read_reg(slg47004_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}


int32_t slg47004_write_reg(slg47004_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//

int32_t slg47004_op_amp_acmp_set(slg47004_ctx_t *ctx, uint8_t VRef0, uint8_t VRef1)
{
	uint8_t wk_buff[8];
	int32_t ret;

	memset(wk_buff, 0, sizeof(wk_buff));

	wk_buff[0] = VRef0;
	ret = slg47004_write_reg(ctx, ACMP_Vref0_ADDR, (uint8_t*)wk_buff, 1);

	wk_buff[0] = VRef1;
	ret = slg47004_write_reg(ctx, ACMP_Vref1_ADDR, (uint8_t*)wk_buff, 1);

	return ret;
}












/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * @file    cmox_low_level_template.c
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "cmox_init.h"
#include "cmox_low_level.h"
#include "stm32f4xx.h"




#define UNUSED(X) (void)X       


#define RCC_CRC_RELEASE_RESET()    (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_CRCRST))




/* Updates the enable state for RCC CRC CLK ENABLE. */
void RCC_CRC_CLK_ENABLE(void)
{
    /* Check the current status before proceeding. */
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN);
	__IO uint32_t tmpreg =  READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN);
	UNUSED(tmpreg);
}

/* CMOX library low level initialization pArg User defined parameter that is transmitted from initialize. */
cmox_init_retval_t cmox_ll_init(void *pArg)
{
    /* Set up the required peripheral configuration. */
  (void)pArg;

   
  RCC_CRC_RELEASE_RESET();
  RCC_CRC_CLK_ENABLE();

  return CMOX_INIT_SUCCESS;
}

/* CMOX library low level de-initialization pArg User defined parameter that is transmitted from finalize. */
cmox_init_retval_t cmox_ll_deInit(void *pArg)
{
    /* Set up the required peripheral configuration. */
  (void)pArg;
   
  return CMOX_INIT_SUCCESS;
}

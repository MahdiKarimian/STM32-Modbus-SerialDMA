/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;
/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit( USHORT usTim1Timerout50us ) //配置50us时钟
{
	
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
	sConfigOC.Pulse = usTim1Timerout50us;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
	return TRUE;;
}


void vMBPortTimersEnable(  ) //打开时钟
{
	/* Clear IT flag */
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
	
	/* Reset counter */
	__HAL_TIM_SET_COUNTER(&htim1,0);

	/* Enable the TIM Update interrupt */
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

	/* Enable the Peripheral */
	__HAL_TIM_ENABLE(&htim1);
}

void vMBPortTimersDisable(  ) //关闭时钟
{
	/* Enable the Peripheral */
	__HAL_TIM_DISABLE(&htim1);

	/* Clear IT flag */
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);

	/* Reset counter */
	__HAL_TIM_SET_COUNTER(&htim1,0);
	
	/* Enable the TIM Update interrupt */
	__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
void prvvTIMERExpiredISR( void ) //在时钟中断内调用
{
    ( void )pxMBPortCBTimerExpired(  );
}


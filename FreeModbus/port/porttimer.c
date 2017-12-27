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
#include "stm32f1xx_hal.h"
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );

static TIM_HandleTypeDef htim;
static uint16_t timeout = 0;
static uint16_t downcounter = 0;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{


    htim.Instance = TIM6;
    htim.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) - 1;;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 50;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    timeout = usTim1Timerout50us;
    
    return HAL_OK == HAL_TIM_Base_Init(&htim) ? TRUE : FALSE;
}


inline void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    downcounter = timeout;
    htim.Instance = TIM6;
    HAL_TIM_Base_Start_IT(&htim);
}

inline void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
    htim.Instance = TIM6;
    HAL_TIM_Base_Stop_IT(&htim);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}

void TIM6_IRQHandler_MB(void)
{
    htim.Instance = TIM6;
    if ((__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_UPDATE) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_UPDATE) != RESET)) {
            __HAL_TIM_CLEAR_IT(&htim, TIM_IT_UPDATE);
            if (!--downcounter)
                prvvTIMERExpiredISR();
        }
}
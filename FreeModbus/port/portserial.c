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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "stm32f1xx_hal.h"
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#define PC_USB 0
#define RS_485 1

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

static UART_HandleTypeDef huart;
static GPIO_TypeDef *DE_Port;
static uint16_t DE_Pin;

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    huart.Instance = USART1;
    if (xRxEnable) {
        __HAL_UART_DISABLE_IT(&huart, UART_IT_TXE);
        __HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);
    }

    if (xTxEnable) {
        __HAL_UART_DISABLE_IT(&huart, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(&huart, UART_IT_TXE);

    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{

    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /*
    switch (ucPORT) {
        case PC_USB:
            huart.Instance = USART1;
            break;
        default:
            return FALSE;
    }
    */
      huart.Instance = USART1;
      huart.Init.BaudRate = 115200;
      huart.Init.WordLength = UART_WORDLENGTH_8B;
      huart.Init.StopBits = UART_STOPBITS_1;
      huart.Init.Parity = UART_PARITY_NONE;
      huart.Init.Mode = UART_MODE_TX_RX;
      huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart.Init.OverSampling = UART_OVERSAMPLING_16;
    /*

    
    switch (ucDataBits) {
        case 8:
            huart.Init.WordLength = UART_WORDLENGTH_8B;
            break;
        case 9:
            huart.Init.WordLength = UART_WORDLENGTH_9B;
            break;
        default:
            return FALSE;
    }
    
    switch (eParity) {
        case MB_PAR_NONE:
            huart.Init.Parity = UART_PARITY_NONE;
            break;
        case MB_PAR_EVEN:
            huart.Init.Parity = UART_PARITY_EVEN;
            break;
        case MB_PAR_ODD:
            huart.Init.Parity = UART_PARITY_ODD;
            break;
        default:
            return FALSE;
    }
    
    */
    return HAL_OK == HAL_UART_Init(&huart);
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */

    huart.Instance->DR = ucByte;
    
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
    huart.Instance = USART1;
    if(huart.Init.Parity == UART_PARITY_NONE)
    {
        *pucByte = (uint8_t)(huart.Instance->DR & (uint8_t)0x00FF);
    
    }
    else
    {

        *pucByte = (uint8_t)(huart.Instance->DR & (uint8_t)0x007F);
    }
    
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}

BOOL UART_IRQ_Handler(USART_TypeDef *usart) {
    huart.Instance = USART1;
    if (huart.Instance == usart) 
    {
        if ((__HAL_UART_GET_FLAG(&huart, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&huart, UART_IT_RXNE) != RESET)) {
                prvvUARTRxISR();
            }
        
        if (( __HAL_UART_GET_FLAG(&huart, UART_FLAG_TXE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&huart, UART_IT_TXE)) != RESET) {
                prvvUARTTxReadyISR();
            }
    }
    
    return FALSE;
}

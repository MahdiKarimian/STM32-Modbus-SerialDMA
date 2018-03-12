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

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#include "stm32f1xx_hal.h"


extern uint8_t usart1_tx_data_buff[1];
extern uint8_t usart1_rx_data_buff[1];
extern UART_HandleTypeDef huart1;

extern void Usart1Pass(void);
/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable ) //控制串口的收发中断
{
	/*Half duplex rs485 interface*/
	if(TRUE==xRxEnable)
	{
		HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
	}
	else
	{
	}

	if(TRUE==xTxEnable)
	{
		HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
	}
}
/*****************************************
* 配置串口 目前除了波特率其他参数无效 
* Usart1 9600-8-N-1
*****************************************/
BOOL 
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{

	HAL_UART_Receive_IT(&huart1, usart1_rx_data_buff, 1);
	//HAL_UART_Transmit_IT(&huart1, usart1_tx_data_buff, 1);
	return TRUE;
}
/*****************************************
*	old serial putbyte, use put buffer instead
*****************************************/

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	volatile int i;
	usart1_tx_data_buff[0] = ucByte;
	HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
	for(i=0; i<1000; i++){}
		HAL_UART_Transmit(&huart1, usart1_tx_data_buff, 1, 1000);
    SET_BIT(huart1.Instance->CR1, USART_CR1_TCIE);		
    return TRUE;
}

/*****************************************
*	xMBPortSerialPutBuff
*	input ptr buffer, length of buffer
*****************************************/
BOOL
xMBPortSerialPutBuff( CHAR * ucBuff,CHAR len )
{
	volatile int i;
	/*Enable transfer with change rs485 pin*/
	HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
	for(i=0; i<1000; i++){}
	/*start dma transfer*/
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *) ucBuff, len);		
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte = huart1.Instance->DR;
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
 void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR( void )
{
     pxMBFrameCBByteReceived(  );
}

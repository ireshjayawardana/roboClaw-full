/*
 * Serial.c
 *
 *  Created on: Aug 27, 2018
 *      Author: sadeesh
 */

#include "Serial.h"

#ifdef USE_USART5
SERIAL_HandleTypeDef __uart5_serial_handler = {

		.buffer_Tx = &USART5_Buffer_TX,
		.buffer_Rx = &USART5_Buffer_RX,
		.hal_reg_Tx = &USART5_HAL_Reg_Tx,
		.hal_reg_Rx = &USART5_HAL_Reg_Rx,
		.application_reg_Rx = &USART5_Application_Reg_Tx,
		.WriteLock = SERIAL_WRITE_UNLOCKED,
		.ReadLock = SERIAL_READ_UNLOCKED
};
#endif
#ifdef USE_USART3
SERIAL_HandleTypeDef __uart3_serial_handler = {

		.buffer_Tx = &USART3_Buffer_TX,
		.buffer_Rx = &USART3_Buffer_RX,
		.hal_reg_Tx = &USART3_HAL_Reg_Tx,
		.hal_reg_Rx = &USART3_HAL_Reg_Rx,
		.application_reg_Rx = &USART3_Application_Reg_Tx,
		.WriteLock = SERIAL_WRITE_UNLOCKED,
		.ReadLock = SERIAL_READ_UNLOCKED
};
#endif

SERIAL_HandleTypeDef* serial_init(UART_HandleTypeDef *huartx) {

	SERIAL_HandleTypeDef *serial_handler = NULL;

#ifdef USE_USART5
		if(huartx->Instance == UART5){

			USART5_Serial_Handler = &(__uart5_serial_handler);

			USART5_Serial_Handler->huartx = huartx;
			ring_buffer_init(USART5_Serial_Handler->buffer_Rx);
			ring_buffer_init(USART5_Serial_Handler->buffer_Tx);
			serial_handler = USART5_Serial_Handler;
			HAL_UART_Receive_DMA(USART5_Serial_Handler->huartx, USART5_Serial_Handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		}
#endif
#ifdef USE_USART3
		if(huartx->Instance == USART3){

			USART3_Serial_Handler = &(__uart3_serial_handler);

			USART3_Serial_Handler->huartx = huartx;
			ring_buffer_init(USART3_Serial_Handler->buffer_Rx);
			ring_buffer_init(USART3_Serial_Handler->buffer_Tx);
			serial_handler = USART3_Serial_Handler;
			HAL_UART_Receive_DMA(USART3_Serial_Handler->huartx, USART3_Serial_Handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		}
#endif

	return serial_handler;

}

void serial_write(SERIAL_HandleTypeDef* hserial, uint8_t *pData, uint16_t len) {

	if (ring_buffer_is_empty(hserial->buffer_Tx)) {

			if (HAL_UART_Transmit_DMA(hserial->huartx, pData, len) != HAL_OK) {
				ring_buffer_queue_arr(hserial->buffer_Tx, pData, len);
			}
	} else {
			ring_buffer_queue_arr(hserial->buffer_Tx, pData, len);
	}
//	ring_buffer_queue_arr(hserial->buffer_Tx, pData, len);
}

uint8_t serial_available(SERIAL_HandleTypeDef* hserial) {

	return !ring_buffer_is_empty(hserial->buffer_Rx);
}

uint8_t serial_read(SERIAL_HandleTypeDef* hserial) {

		if (!ring_buffer_is_empty(hserial->buffer_Rx)) {
			ring_buffer_dequeue(hserial->buffer_Rx, hserial->application_reg_Rx);
			return *(hserial->application_reg_Rx);
		}

		return '\0';
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	SERIAL_HandleTypeDef* serial_handler = get_serial_handler(huart);

	if (!ring_buffer_is_empty(serial_handler->buffer_Tx)) {

		  ring_buffer_dequeue(serial_handler->buffer_Tx, serial_handler->hal_reg_Tx);
		  HAL_UART_Transmit_DMA(huart, serial_handler->hal_reg_Tx, PRIMARY_REG_SIZE);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	SERIAL_HandleTypeDef* serial_handler = get_serial_handler(huart);

	ring_buffer_queue_arr(serial_handler->buffer_Rx, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
	HAL_UART_Receive_DMA(huart, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->ErrorCode == HAL_UART_ERROR_ORE) {

	}
}


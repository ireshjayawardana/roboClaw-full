/*
 * base_serial.h
 *
 *  Created on: Aug 30, 2018
 *      Author: sadeesh
 */

#ifndef SERIAL_BASE_SERIAL_H_
#define SERIAL_BASE_SERIAL_H_

#include "ringbuffer.h"
#include "usart.h"

#define USE_USART5
#define USE_USART3

#define USART5_DMA

typedef enum {
	SERIAL_OK = 0x00U,
	SERIAL_ERROR = 0x01U,
	SERIAL_BUSY = 0x02U,
	SERIAL_TIMEOUT = 0x03U
} SERIAL_StatusTypeDef;

typedef enum {
	SERIAL_WRITE_UNLOCKED = 0x00U, SERIAL_WRITE_LOCKED = 0x01U
} SERIAL_WRITE_LockTypeDef;

typedef enum {
	SERIAL_READ_UNLOCKED = 0x00U, SERIAL_READ_LOCKED = 0x01U
} SERIAL_READ_LockTypeDef;

typedef enum {
	Serial_OK = 0x00U, Serial_EMPTY = 0x01U
} Serial_StatusTypeDef;

typedef struct{

	UART_HandleTypeDef *huartx; 			/*!< UART handler address                               */
	ring_buffer_t *buffer_Tx;				/*!<  Pointer to UART Tx software ring buffer           */
	ring_buffer_t *buffer_Rx;				/*!< Pointer to UART Rx software ring buffer            */
	uint8_t *hal_reg_Tx;					/*!< Pointer to utility reg to HAL in Tx                */
	uint8_t *hal_reg_Rx;					/*!< Pointer to utility reg to HAL in Rx                */
	uint8_t *application_reg_Rx;			/*!< Pointer to utility reg to application layer in Rx  */

	SERIAL_WRITE_LockTypeDef WriteLock;
	SERIAL_READ_LockTypeDef ReadLock;

} SERIAL_HandleTypeDef;

#define PRIMARY_REG_SIZE 1

/** Define variables for Serial handlers according to each USART peripheral */

#ifdef USE_USART5
SERIAL_HandleTypeDef* USART5_Serial_Handler;
ring_buffer_t USART5_Buffer_TX, USART5_Buffer_RX;
uint8_t USART5_HAL_Reg_Tx, USART5_HAL_Reg_Rx, USART5_Application_Reg_Tx;
#endif
#ifdef USE_USART3
SERIAL_HandleTypeDef* USART3_Serial_Handler;
ring_buffer_t USART3_Buffer_TX, USART3_Buffer_RX;
uint8_t USART3_HAL_Reg_Tx, USART3_HAL_Reg_Rx, USART3_Application_Reg_Tx;
#endif

SERIAL_HandleTypeDef* get_serial_handler(UART_HandleTypeDef *huartx);

#endif /* SERIAL_BASE_SERIAL_H_ */

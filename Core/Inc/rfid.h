#ifndef __RFID_H__
#define __RFID_H__

#include "main.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief Initialize the RFID reader (prints startup message)
 * @param rfid_uart  UART handle connected to the RFID module (e.g., &huart1)
 * @param debug_uart UART handle for PC debug output (e.g., &huart2)
 */
void RFID_Init(UART_HandleTypeDef *rfid_uart, UART_HandleTypeDef *debug_uart);

/**
 * @brief Reads an RFID tag and checks authorization
 * @param rfid_uart  UART handle connected to the RFID module
 * @param debug_uart UART handle for debug/print output
 * @param tagBuffer  Pointer to store the read tag string (null-terminated)
 * @retval 1 if authorized tag detected
 * @retval 2 if unauthorized tag detected
 * @retval 0 if no tag or error
 */
uint8_t RFID_ReadTag(UART_HandleTypeDef *rfid_uart, UART_HandleTypeDef *debug_uart, char *tagBuffer);

#endif /* __RFID_H__ */

/*
 * bootloader.h
 *
 *  Created on: Nov 14, 2022
 *      Author: 150100
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include "main.h"

//macros for UARt bootloader command
#define BL_UART_SOF			0x7F
#define GET_CMD_COMMAND		0x00
#define GET_VER_COMMAND		0x01
#define EMEM_COMMAND		0x44
#define WMEM_COMMAND		0x31
#define RMEM_COMMAND		0x11



#endif /* INC_BOOTLOADER_H_ */

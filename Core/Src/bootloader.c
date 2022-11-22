/*
 * bootloader.c
 *
 *  Created on: Nov 14, 2022
 *      Author: 150100
 */

#include "bootloader.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

uint8_t receive_cmd_response = {0};
uint8_t invoke_uart = 0x7f;

void target_reset(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

static uint8_t xor_checksum(const uint8_t pData[], uint16_t len)
{
  uint8_t sum = 0;

  for (uint16_t i = 0U; i < len; i++)
  {
    sum ^= pData[i];
  }

  return sum;
}

//enter to bootloader mode(target - H7A3)
void enter_boot(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //boot0 pin of H7A3 bring to high
}

//exit from bootloader mode(target - H7A3)
void exit_boot(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //boot0 pin of H7A3 bring to low
}

//bootloader invoke UART command
void BL_Invoke_UART(void){
	HAL_UART_Transmit(&huart1, &invoke_uart, 1, 100);
	wait_for_ack_byte(&huart1, &receive_cmd_response); //receive ACK/NACK byte
}

//receive the ACK/NACK byte
void wait_for_ack_byte(UART_HandleTypeDef *huart1, uint8_t *recv){
	HAL_UART_Receive(huart1, recv, 1, 1000); //receive ACK/NACK byte
}

//bootloader get command
void BL_Get_Cmd(uint8_t *pData){
	  uint8_t cmd_frame[2];
	  uint8_t rx_number_of_bytes;

//	  cmd_frame[0] = BL_UART_SOF;
	  cmd_frame[0] = GET_CMD_COMMAND;
	  cmd_frame[1] = GET_CMD_COMMAND ^ 0xFFU; /*!< Command XOR checksum */
	  HAL_UART_Transmit(&huart1, cmd_frame, 2, 1000);

	  wait_for_ack_byte(&huart1, &receive_cmd_response); /*!< wait for ACK/NACK byte*/

	  HAL_UART_Receive(&huart1, &rx_number_of_bytes, 1, 1000); /*!< receive the length of data to be receive*/

	  HAL_UART_Receive(&huart1, pData, (uint16_t)(rx_number_of_bytes+1), 1000);
	  HAL_UART_Transmit(&huart2, pData, (uint16_t)(rx_number_of_bytes+1), 1000);
}

//bootloader get version command
uint8_t BL_GetVersion_Command(void)
{
  uint8_t cmd_frame[2];
  uint8_t version = 0x00U;

  /* Send start of frame (0x7F) + Get Version command frame (0x01 0xEE) */
//  cmd_frame[0] = BL_UART_SOF;
  cmd_frame[0] = GET_VER_COMMAND;
  cmd_frame[1] = GET_VER_COMMAND ^ 0xFFU;
  HAL_UART_Transmit(&huart1, cmd_frame, 2, 1000);

  /* Wait for ACK or NACK frame */
  wait_for_ack_byte(&huart1, &receive_cmd_response); /*!< wait for ACK/NACK byte*/

  /* Receive data frame */
  wait_for_ack_byte(&huart1, &receive_cmd_response); /*!< wait for ACK/NACK byte*/
  HAL_UART_Receive(&huart1, &version, 1, 1000);

  return version;
}

//clear flash memory of target
void BL_EraseMemory_Command(uint16_t nob, uint8_t code){
	  uint8_t cmd_frame[2];
	  uint8_t data_frame[3];
	  uint8_t global_erase = 0xFF;
	  uint8_t no_sec_erase = nob - 1;

	  cmd_frame[0] = EMEM_COMMAND;
	  cmd_frame[1] = EMEM_COMMAND ^ 0xFFU;

	  HAL_UART_Transmit(&huart1, cmd_frame, 2, 1000);
	  wait_for_ack_byte(&huart1, &receive_cmd_response); /*!< wait for ACK/NACK byte*/
	  HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);

	  if(nob == 0x00FF){
		  HAL_UART_Transmit(&huart1, &global_erase, 1, 1000); //send 0xff to request the global erase
		  data_frame[0] = global_erase; //0xFF - for global erase
		  data_frame[1] = 0x00; //0x00 - for global erase
		  data_frame[2] = data_frame[0] ^ data_frame[1];
	  } else{
		  HAL_UART_Transmit(&huart1, &no_sec_erase, 1, 1000); //send 0xff to request the global erase
		  data_frame[0] = nob; //how much sector to be erased
		  data_frame[1] = code; //from where to erase
		  data_frame[2] = data_frame[0] ^ data_frame[1];
	  }


	  HAL_UART_Transmit(&huart1, data_frame, 3, 1000);

	  wait_for_ack_byte(&huart1, &receive_cmd_response); /*!< wait for ACK/NACK byte*/
	  HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);
}

//bootloader write to flash command
void BL_WriteMemory_Command(uint32_t address, uint16_t nob, uint8_t *pData){
	  int temp = 256;
	  uint32_t addr_to_write = address;
	  uint16_t byte_to_write = 0x0100; //max 256 bytes can be written at a time
	  uint16_t bytes_remains_to_w = nob;
	  uint8_t checksum;
	  int where = 0;

	  uint8_t cmd_frame[2];
	  uint8_t addr_frame[5];
	  uint8_t n = 0xFF;

	  cmd_frame[0] = WMEM_COMMAND;
	  cmd_frame[1] = WMEM_COMMAND ^ 0xFFU;

	  while(bytes_remains_to_w != 0x0000){

		HAL_UART_Transmit(&huart1, cmd_frame, 2, 1000);
		HAL_UART_Receive(&huart1, &receive_cmd_response, 1, 100); //receive the ACK/NACK byte
		HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);

		addr_frame[0] = ((uint8_t) (addr_to_write >> 24) & 0xFFU);
		addr_frame[1] = ((uint8_t) (addr_to_write >> 16) & 0xFFU);
		addr_frame[2] = ((uint8_t) (addr_to_write >> 8) & 0xFFU);
		addr_frame[3] = ((uint8_t) addr_to_write & 0xFFU);
		addr_frame[4] = xor_checksum(addr_frame, 4U);

		HAL_UART_Transmit(&huart1, addr_frame, 5, 1000);
		HAL_UART_Receive(&huart1, &receive_cmd_response, 1, 100); //receive the ACK/NACK byte
		HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);

		if(bytes_remains_to_w < byte_to_write){
			n = bytes_remains_to_w - 1;
			HAL_UART_Transmit(&huart1, (uint8_t *)&n, 1, 1000);
			checksum = xor_checksum(&pData[where], bytes_remains_to_w) ^ n;
			HAL_UART_Transmit(&huart1, &pData[where], bytes_remains_to_w, 1000);
			bytes_remains_to_w = 0x0000;
		} else{
			HAL_UART_Transmit(&huart1, (uint8_t *)&n, 1, 1000);
			checksum = xor_checksum(&pData[where], byte_to_write) ^ n;
			HAL_UART_Transmit(&huart1, &pData[where], byte_to_write, 1000);
			if(receive_cmd_response == 0x79){
				bytes_remains_to_w -= (byte_to_write); //subtract 4(byte_to_write) eachtime from the bytes_remain_to_w we write to the flash
			}
		}

		HAL_UART_Transmit(&huart1, &checksum, 1, 1000);
		HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);

		HAL_UART_Receive(&huart1, &receive_cmd_response, 1, 1000); //receive the ACK/NACK byte
		HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);
		if(receive_cmd_response == 0x79)
		{
			addr_to_write += (uint32_t)(byte_to_write); //increment address by 4(byte_to_write) bytes once we transfer the 32bits of data
			where += temp;
		//	HAL_UART_Transmit(&huart2, (uint8_t *)&addr_to_write, 4, 100);
		}
	  }
}

//read data from memory
void BL_ReadMemory_Command(uint32_t address, uint8_t nob, uint8_t *pData){
	  uint8_t cmd_frame[2];
	  uint8_t addr_frame[5];
//	  uint32_t address = 0x08000000;
	  uint8_t nob_frame[2];
//	  uint8_t nob = 4;

	  cmd_frame[0] = RMEM_COMMAND;
	  cmd_frame[1] = RMEM_COMMAND ^ 0xFFU;
	  HAL_UART_Transmit(&huart1, cmd_frame, 2, 1000);
	  wait_for_ack_byte(&huart1, &receive_cmd_response); /*!< wait for ACK/NACK byte*/
	  HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);

	  addr_frame[0] = (uint8_t) ((address >> 24) & 0xFFU);
	  addr_frame[1] = (uint8_t) ((address >> 16) & 0xFFU);
	  addr_frame[2] = (uint8_t) ((address >> 8) & 0xFFU);
	  addr_frame[3] = (uint8_t) (address & 0xFFU);
	  addr_frame[4] = xor_checksum(addr_frame, 4U);

	  HAL_UART_Transmit(&huart1, addr_frame, 5, 1000);

	  wait_for_ack_byte(&huart1, &receive_cmd_response); /*!< wait for ACK/NACK byte*/
	  HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);

	  nob_frame[0] = (nob - 1U);
	  nob_frame[1] = (nob - 1U) ^ 0xFFU;

	  HAL_UART_Transmit(&huart1, nob_frame, 2, 1000);
	  wait_for_ack_byte(&huart1, &receive_cmd_response); /*!< wait for ACK/NACK byte*/
	  HAL_UART_Transmit(&huart2, &receive_cmd_response, 1, 100);

	  HAL_UART_Receive(&huart1, pData, (uint16_t)nob, 1000);
	  HAL_UART_Transmit(&huart2, pData, (uint16_t)nob, 1000);
  }

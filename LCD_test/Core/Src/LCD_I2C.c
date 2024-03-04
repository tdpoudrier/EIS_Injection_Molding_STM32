/*
 * LCD_I2C.c
 *
 *  Created on: Mar 3, 2024
 *      Author: Tevin Poudrier
 */

#include "LCD_I2C.h"

/**
 * Set MCP23008 GPIO to output and then initialize LCD
 */
HAL_StatusTypeDef LCD_Init (LCD_HandleTypeDef *hlcd, I2C_HandleTypeDef *hi2c, uint8_t address) {
	hlcd->hi2c = hi2c;
	hlcd->address = address << 1;

	HAL_StatusTypeDef status = HAL_OK;

	//Set GPIO direction to output
	status = SendCommand(hlcd, MCP23008_IODIR, 0x00);
	if (status != HAL_OK) {return status;}


	return status;
}

HAL_StatusTypeDef LCD_EnableBacklight (LCD_HandleTypeDef *hlcd) {
	return SendCommand (hlcd, MCP23008_GPIO, LCD_BACKLIGHT);
}

HAL_StatusTypeDef LCD_DisableBacklight (LCD_HandleTypeDef *hlcd) {
	return SendCommand (hlcd, MCP23008_GPIO, 0x00);
}


HAL_StatusTypeDef SendCommand (LCD_HandleTypeDef *hlcd, uint8_t mcp23008Address, uint8_t data) {
	uint8_t dataBuffer[] = {mcp23008Address, data};
	return HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->address, dataBuffer, sizeof(dataBuffer), 1000);
}



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
	hlcd->backlightOn = 1;

	HAL_StatusTypeDef status = HAL_OK;

	//Set GPIO direction to output
	status = SendCommand(hlcd, MCP23008_IODIR, 0x00);
	if (status != HAL_OK) {return status;}

	//Wake up
	LCD_SendCommand(hlcd, 0x18);
	HAL_Delay(50);
	LCD_SendCommand(hlcd, 0x18);
	HAL_Delay(50);
	LCD_SendCommand(hlcd, 0x18);
	HAL_Delay(50);

	//Set LCD to 4 bits
	LCD_SendCommand(hlcd, 0x10);

	//Set LCD to 16x2
	LCD_SendCommand(hlcd, 0x10);
	LCD_SendCommand(hlcd, 0x60);

	//Display off
	LCD_SendCommand(hlcd, 0x00);
	LCD_SendCommand(hlcd, 0x40);

	//Display clear
	LCD_SendCommand(hlcd, 0x00);
	LCD_SendCommand(hlcd, 0x08);

	//Entry mode set
	LCD_SendCommand(hlcd, 0x00);
	LCD_SendCommand(hlcd, 0x38);

	//Turn on display, enable blinking cursor
	LCD_SendCommand(hlcd, 0x00);
	LCD_SendCommand(hlcd, 0x78);

	//Return home
	LCD_SendCommand(hlcd, 0x00);
	LCD_SendCommand(hlcd, 0x10);

	//Write A
	LCD_SendCommand(hlcd, 0x20 | 0x02);
	LCD_SendCommand(hlcd, 0x08 | 0x02);

	//Write B
	LCD_SendCommand(hlcd, 0x20 | 0x02);
	LCD_SendCommand(hlcd, 0x10 | 0x02);

	return status;
}

HAL_StatusTypeDef LCD_EnableBacklight (LCD_HandleTypeDef *hlcd) {
	hlcd->backlightOn = 1;
	return SendCommand (hlcd, MCP23008_GPIO, LCD_BACKLIGHT);
}

HAL_StatusTypeDef LCD_DisableBacklight (LCD_HandleTypeDef *hlcd) {
	hlcd->backlightOn = 0;
	return SendCommand (hlcd, MCP23008_GPIO, 0x00);
}

HAL_StatusTypeDef LCD_SendCommand (LCD_HandleTypeDef *hlcd, uint8_t data) {
	HAL_StatusTypeDef status = HAL_OK;

	//set enable high
	status = SendCommand(hlcd, MCP23008_GPIO, LCD_ENABLE);
	if (status != HAL_OK) {return status;}

	//Send data
	status = SendCommand(hlcd, MCP23008_GPIO, data | LCD_ENABLE | LCD_BACKLIGHT);
	if (status != HAL_OK) {return status;}

	//set enable low
	status = SendCommand(hlcd, MCP23008_GPIO, 0x00 | LCD_BACKLIGHT);
	if (status != HAL_OK) {return status;}

	HAL_Delay(10);

	return status;
}

HAL_StatusTypeDef SendCommand (LCD_HandleTypeDef *hlcd, uint8_t mcp23008Address, uint8_t data) {
	uint8_t dataBuffer[] = {mcp23008Address, data};
	return HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->address, dataBuffer, sizeof(dataBuffer), 1000);
}





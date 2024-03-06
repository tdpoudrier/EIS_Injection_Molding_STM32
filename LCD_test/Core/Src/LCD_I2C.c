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
	LCD_Write(hlcd, 0x18);
	HAL_Delay(50);
	LCD_Write(hlcd, 0x18);
	HAL_Delay(50);
	LCD_Write(hlcd, 0x18);
	HAL_Delay(50);

	//Set LCD to 4 bits
	LCD_Write(hlcd, 0x10);

	//function set 2-line, 5x10 dots
	LCD_SendCommand(hlcd, 0x2B, true);

	//Display off
	LCD_SendCommand(hlcd, 0x08, true);

	//display clear
	LCD_SendCommand(hlcd, 0x01, true);

	//entry mode
	LCD_SendCommand(hlcd, 0x06, true);

	//Turn display on and blink cursor
	LCD_SendCommand(hlcd, 0x0f, true);

//	//Set cursor to move and shift right
//	LCD_SendCommand(hlcd, 0x1B, true);

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


//Convert 8 bit command into 4 bit command
HAL_StatusTypeDef LCD_SendCommand (LCD_HandleTypeDef *hlcd, uint8_t data, uint8_t isInstruction) {

	HAL_StatusTypeDef status = HAL_OK;

	//Seperate into Nibbles
	uint8_t upperNibble = (data & 0xF0) >> 4;
	uint8_t lowNibble = data & 0x0F;

	//Calculate rs bit
	uint8_t rsBit = (!isInstruction & 0x01) << 1;

	//Send data to MCP23008 GPIO
	uint8_t data1 = (upperNibble << 3) | rsBit;
	uint8_t data2 = (lowNibble << 3) | rsBit;
	status = LCD_Write(hlcd, data1);
	status = LCD_Write(hlcd, data2);

	return status;
}

HAL_StatusTypeDef LCD_Write (LCD_HandleTypeDef *hlcd, uint8_t data) {
	HAL_StatusTypeDef status = HAL_OK;

	//Send data
	status = SendCommand(hlcd, MCP23008_GPIO, data | LCD_BACKLIGHT);
	if (status != HAL_OK) {return status;}

	HAL_Delay(10);

	//Sent enable high
	status = SendCommand(hlcd, MCP23008_GPIO, data | LCD_ENABLE | LCD_BACKLIGHT);
	if (status != HAL_OK) {return status;}

	//set enable low
	status = SendCommand(hlcd, MCP23008_GPIO, data | LCD_BACKLIGHT);
	if (status != HAL_OK) {return status;}

	HAL_Delay(10);

	return status;
}

HAL_StatusTypeDef SendCommand (LCD_HandleTypeDef *hlcd, uint8_t mcp23008Address, uint8_t data) {
	uint8_t dataBuffer[] = {mcp23008Address, data};
	return HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->address, dataBuffer, sizeof(dataBuffer), 1000);
}





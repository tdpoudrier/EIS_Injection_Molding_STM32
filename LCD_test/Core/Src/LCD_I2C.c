/*
 * LCD_I2C.c
 *
 *  Created on: Mar 3, 2024
 *      Author: Tevin Poudrier
 */

#include "LCD_I2C.h"

/**
 * CALL AFTER HAL_I2C_Init();
 * Initialize LCD to communicate with 4-bit interface at given address and communicates over i2c. See page 46 of HD44780 datasheet.
 * @param hlcd Pointer to handle for LCD
 * @param hi2c Pointer to handle for I2C communication, this is auto-generated by CubeMX
 * @param address Address of LCD I2C backpack as specified in the datasheet (MCP23008)
 * @return HAL_OK if data was sent over I2C successfully
 */
HAL_StatusTypeDef LCD_Init (LCD_HandleTypeDef *hlcd, I2C_HandleTypeDef *hi2c, uint8_t address) {
	hlcd->hi2c = hi2c;
	hlcd->address = address << 1;

	HAL_StatusTypeDef status = HAL_OK;

	//Set MCP23008 GPIO Register direction to output
	status += MCP23008_SendDataI2C(hlcd, MCP23008_IODIR, 0x00);

	//Wait for VDD
	HAL_Delay(50);

	//Wake up, Enter initalization
	status += LCD_SendData(hlcd, 0x18);
	HAL_Delay(50);
	status += LCD_SendData(hlcd, 0x18);
	HAL_Delay(50);
	status += LCD_SendData(hlcd, 0x18);
	HAL_Delay(50);

	//Set LCD to 4 bits
	status += LCD_SendData(hlcd, 0x10);

	//function set 2-line, 5x10 dots
	status += LCD_WriteCommand(hlcd, 0x2B);

	//Display off
	status += LCD_WriteCommand(hlcd, 0x08);

	//display clear
	status += LCD_WriteCommand(hlcd, 0x01);

	//entry mode
	status += LCD_WriteCommand(hlcd, 0x06);

	//Turn display on and blink cursor
	status += LCD_WriteCommand(hlcd, 0x0f);

	return status;
}

HAL_StatusTypeDef LCD_Clear (LCD_HandleTypeDef *hlcd) {
	return LCD_WriteCommand(hlcd, 0x01);
}

/**
 * Sets the cursor of the LCD at the given column and row
 * @param hlcd Pointer to LCD handle
 * @param col Column position of cursor, ranges from 0-19
 * @param row Row position of cursor, ranges from 0-3
 * @return HAL_OK on success, otherwise HAL_ERROR
 */
HAL_StatusTypeDef LCD_SetCursor (LCD_HandleTypeDef *hlcd, uint8_t col, uint8_t row) {
	if (col > 19) {
		return HAL_ERROR;
	}

	uint8_t cursorPos = 0;

	switch (row) {
	case 0:
		cursorPos = col + LCD_ROW_0;
		break;
	case 1:
		cursorPos = col + LCD_ROW_1;
		break;
	case 2:
		cursorPos = col + LCD_ROW_2;
		break;
	case 3:
		cursorPos = col + LCD_ROW_3;
		break;
	default:
		return HAL_ERROR;
	}

	return LCD_WriteCommand(hlcd, LCD_SET_DDRAM | cursorPos);
}

HAL_StatusTypeDef LCD_Print (LCD_HandleTypeDef *hlcd, char *string) {
	HAL_StatusTypeDef status = HAL_OK;
	for (int i = 0; i < strlen(string); i++) {
		status += LCD_PrintChar(hlcd, string[i]);
	}
	return status;
}


/**
 * Convert 8 bit command into two 4-bit command and write to LCD
 * @param hlcd Pointer to LCD handle
 * @param byte LCD 8-bit command
 * @return HAL_OK if successful, number of errors otherwise
 */
HAL_StatusTypeDef LCD_WriteCommand (LCD_HandleTypeDef *hlcd, uint8_t byte) {

	HAL_StatusTypeDef status = HAL_OK;

	//Separate into 4-bit Nibbles
	uint8_t upperNibble = (byte & 0xF0) >> 4;
	uint8_t lowNibble = byte & 0x0F;

	//Send data to LCD
	status += LCD_SendData(hlcd, upperNibble << 3);
	status += LCD_SendData(hlcd, lowNibble << 3);

	return status;
}

/**
 * Write character to LCD by converting 8-bit char to two 4-bit nibbles
 * @param hlcd Pointer to handle for LCD
 * @param letter Unsigned character to print
 * @return
 */
HAL_StatusTypeDef LCD_PrintChar (LCD_HandleTypeDef *hlcd, unsigned char letter) {

	HAL_StatusTypeDef status = HAL_OK;

	//Separate into Nibbles
	uint8_t upperNibble = (letter & 0xF0) >> 4;
	uint8_t lowNibble = letter & 0x0F;

	//Send data to LCD
	status += LCD_SendData(hlcd, (upperNibble << 3) | LCD_WRITE_DATA);
	status += LCD_SendData(hlcd, (lowNibble << 3) | LCD_WRITE_DATA);

	return status;
}

/**
 * Send the data to the LCD, pulses the Enable pin so LCD receives data
 * @param hlcd Pointer to handle for LCD
 * @param data Data to be sent to the LCD
 * @return HAL_OK on success, stops and returns HAL_ERROR otherwise
 */
HAL_StatusTypeDef LCD_SendData (LCD_HandleTypeDef *hlcd, uint8_t data) {
	HAL_StatusTypeDef status = HAL_OK;

	//Send data
	status = MCP23008_SendDataI2C(hlcd, MCP23008_GPIO, data | LCD_BACKLIGHT);
	if (status != HAL_OK) {return status;}

	HAL_Delay(1);

	//Sent enable high
	status = MCP23008_SendDataI2C(hlcd, MCP23008_GPIO, data | LCD_ENABLE | LCD_BACKLIGHT);
	if (status != HAL_OK) {return status;}

	HAL_Delay(1);

	//set enable low
	status = MCP23008_SendDataI2C(hlcd, MCP23008_GPIO, data | LCD_BACKLIGHT);
	if (status != HAL_OK) {return status;}

	HAL_Delay(1);

	return status;
}

/**
 * Send the data over I2C to the MCP23008 serial to parellel converter.
 * @param hlcd Pointer to handle for LCD
 * @param mcp23008Address Register address for MCP23008
 * @param data Data to be sent to MCP23008 over I2C
 * @return
 */
HAL_StatusTypeDef MCP23008_SendDataI2C (LCD_HandleTypeDef *hlcd, uint8_t mcp23008Address, uint8_t data) {
	uint8_t dataBuffer[] = {mcp23008Address, data};
	return HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->address, dataBuffer, sizeof(dataBuffer), 1000);
}

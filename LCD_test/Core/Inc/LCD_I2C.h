/*
 * LCD_I2C.h
 *
 *  Created on: Mar 3, 2024
 *      Author: Tevin Poudrier
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include "stm32c0xx_hal.h"
#include <stdbool.h>

#define MCP23008_GPIO 0x9
#define MCP23008_IODIR 0x0

#define LCD_BACKLIGHT 0x80
#define LCD_ENABLE 0x04

typedef struct __LCD_HandleTypeDef {
	uint16_t address;
	I2C_HandleTypeDef *hi2c;
	uint8_t backlightOn;
} LCD_HandleTypeDef;

HAL_StatusTypeDef LCD_Init (LCD_HandleTypeDef *hlcd, I2C_HandleTypeDef *hi2c, uint8_t address);

HAL_StatusTypeDef SendCommand (LCD_HandleTypeDef *hlcd, uint8_t mcp23008Address, uint8_t data);

HAL_StatusTypeDef LCD_Write (LCD_HandleTypeDef *hlcd, uint8_t data);

HAL_StatusTypeDef LCD_SendCommand (LCD_HandleTypeDef *hlcd, uint8_t data, uint8_t isInstruction);


#endif /* INC_LCD_I2C_H_ */

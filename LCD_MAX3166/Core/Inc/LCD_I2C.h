/*
 * LCD_I2C.h
 *
 *  Created on: Mar 3, 2024
 *      Author: Tevin Poudrier
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include "stm32c0xx_hal.h"
#include <string.h>

#define MCP23008_GPIO 0x9
#define MCP23008_IODIR 0x0

#define LCD_BACKLIGHT 0x80
#define LCD_ENABLE 0x04
#define LCD_WRITE_DATA 0x02
#define LCD_SET_DDRAM 0x80

#define LCD_ROW_0 0x00
#define LCD_ROW_1 0x40
#define LCD_ROW_2 0x14
#define LCD_ROW_3 0x54

typedef struct __LCD_HandleTypeDef {
	uint16_t address;
	I2C_HandleTypeDef *hi2c;
} LCD_HandleTypeDef;

HAL_StatusTypeDef LCD_Init (LCD_HandleTypeDef *hlcd, I2C_HandleTypeDef *hi2c, uint8_t address);

HAL_StatusTypeDef LCD_Clear (LCD_HandleTypeDef *hlcd);

HAL_StatusTypeDef LCD_SetCursor (LCD_HandleTypeDef *hlcd, uint8_t col, uint8_t row);

HAL_StatusTypeDef LCD_Print (LCD_HandleTypeDef *hlcd, char *string);

HAL_StatusTypeDef LCD_WriteCommand (LCD_HandleTypeDef *hlcd, uint8_t data);

HAL_StatusTypeDef LCD_PrintChar (LCD_HandleTypeDef *hlcd, uint8_t data);

HAL_StatusTypeDef LCD_SendData (LCD_HandleTypeDef *hlcd, uint8_t data);

HAL_StatusTypeDef MCP23008_SendDataI2C (LCD_HandleTypeDef *hlcd, uint8_t mcp23008Address, uint8_t data);


#endif /* INC_LCD_I2C_H_ */

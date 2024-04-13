/*
 * LCD_Menu_Data.h
 *
 *  Created on: Apr 6, 2024
 *      Author: Tevin Poudrier
 */

#ifndef INC_LCD_MENU_DATA_H_
#define INC_LCD_MENU_DATA_H_

#include "LCD_I2C.h"

#define LCD_DATA_COL 17

typedef struct __LCD_MENU_Data {
	LCD_HandleTypeDef* hlcd;
	uint16_t value;
	uint8_t rowPos;
	uint8_t colPos;
} LCD_MENU_Data;

void LCD_MENU_DataInit (LCD_MENU_Data* dataItem, LCD_HandleTypeDef* hlcd);

void LCD_MENU_DataIncrement (LCD_MENU_Data* dataItem, uint8_t direction);

void LCD_MENU_DataSetValue (LCD_MENU_Data* dataItem, uint16_t value);

void LCD_MENU_DataToggle (LCD_MENU_Data* dataItem);

void LCD_MENU_DataPrintBool (LCD_MENU_Data* dataItem, uint8_t row);

void LCD_MENU_DataPrint (LCD_MENU_Data* dataItem, uint8_t row);




#endif /* INC_LCD_MENU_DATA_H_ */

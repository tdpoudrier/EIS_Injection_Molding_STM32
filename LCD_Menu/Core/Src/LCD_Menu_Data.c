/*
 * LCD_Menu_Data.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Tevin Poudrier
 */


#include "LCD_Menu_Data.h"
#include "stdio.h"

void LCD_MENU_DataInit (LCD_MENU_Data* dataItem, LCD_HandleTypeDef* hlcd, uint8_t lcd_row, uint8_t lcd_col) {
	dataItem->hlcd = hlcd;
	dataItem->rowPos = lcd_row;
	dataItem->colPos = lcd_col;
	dataItem->value = 0;
}

void LCD_MENU_DataIncrement (LCD_MENU_Data* dataItem, uint8_t direction) {
	if (direction == 1) {
		dataItem->value++;
	}
	else {
		dataItem->value--;
	}
	if(dataItem->value > 999) {
		dataItem->value = 0;
	}
	LCD_MENU_DataPrint (dataItem);
}

void LCD_MENU_DataSetValue (LCD_MENU_Data* dataItem, uint16_t value) {
	dataItem->value = value;

	if(dataItem->value > 999) {
		dataItem->value = 0;
	}

	LCD_MENU_DataPrint (dataItem);
}

void LCD_MENU_DataPrint (LCD_MENU_Data* dataItem) {
	char str_buff[21] = {'\0'};
	snprintf(str_buff, 21, "%03d", dataItem->value);
    LCD_SetCursor(dataItem->hlcd, dataItem->colPos, dataItem->rowPos);
    LCD_Print(dataItem->hlcd, str_buff);
}

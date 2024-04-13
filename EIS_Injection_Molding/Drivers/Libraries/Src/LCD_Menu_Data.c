/*
 * LCD_Menu_Data.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Tevin Poudrier
 */


#include "LCD_Menu_Data.h"
#include "stdio.h"

void LCD_MENU_DataInit (LCD_MENU_Data* dataItem, LCD_HandleTypeDef* hlcd) {
	dataItem->hlcd = hlcd;
	dataItem->rowPos = -1;
	dataItem->colPos = LCD_DATA_COL;
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
}

void LCD_MENU_DataSetValue (LCD_MENU_Data* dataItem, uint16_t value) {
	dataItem->value = value;

	if(dataItem->value > 999) {
		dataItem->value = 0;
	}
}

void LCD_MENU_DataToggle (LCD_MENU_Data* dataItem) {
	if (dataItem->value > 0) {
		dataItem->value = 0;
	}
	else {
		dataItem->value = 1;
	}
}

void LCD_MENU_DataPrintBool (LCD_MENU_Data* dataItem, uint8_t row) {
	LCD_SetCursor(dataItem->hlcd, LCD_DATA_COL, row);
	if (dataItem->value > 0) {
		LCD_Print(dataItem->hlcd, "ON ");
	}
	else {
		LCD_Print(dataItem->hlcd, "OFF");
	}
}

void LCD_MENU_DataPrint (LCD_MENU_Data* dataItem, uint8_t row) {
	char str_buff[21] = {'\0'};
	snprintf(str_buff, 21, "%03d", dataItem->value);
    LCD_SetCursor(dataItem->hlcd, LCD_DATA_COL, row);
    LCD_Print(dataItem->hlcd, str_buff);
}

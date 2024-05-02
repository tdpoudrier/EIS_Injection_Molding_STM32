/*
 * LCD_Menu_Data.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Tevin Poudrier
 *      Description: data display element for LCD menu interface. Allows data to be printed and modified while displayed on LCD
 */


#include "LCD_Menu_Data.h"
#include "stdio.h"

/**
 * Initialize the data menu element with LCD handle and initialize variables
 * @param hlcd Pointer to LCD handle
 * @param dataItem Pointer to the menu data
 */
void LCD_MENU_DataInit (LCD_MENU_Data* dataItem, LCD_HandleTypeDef* hlcd) {
	dataItem->hlcd = hlcd;
	dataItem->rowPos = -1;
	dataItem->colPos = LCD_DATA_COL;
	dataItem->value = 0;
}

/**
 * Increment value stored in dataItem
 * @param dataItem Pointer to the menu data being modified
 * @param direction The direction the data value is incremented, 1 for increasing and 0 otherwise
 */
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

/**
 * Set the data value
 * @param dataItem Pointer to the menu data being modified
 * @param value The value to be stored
 */
void LCD_MENU_DataSetValue (LCD_MENU_Data* dataItem, uint16_t value) {
	dataItem->value = value;

	if(dataItem->value > 999) {
		dataItem->value = 0;
	}
}

/**
 * Toggle the data between 0 and 1
 * @param dataItem Pointer to the menu data being modified
 */
void LCD_MENU_DataToggle (LCD_MENU_Data* dataItem) {
	if (dataItem->value > 0) {
		dataItem->value = 0;
	}
	else {
		dataItem->value = 1;
	}
}

/**
 * Print ON or OFF at row position if dataItem value is greater than one
 * @param dataItem Pointer to the menu data
 * @param row The row to print to the LCD
 */
void LCD_MENU_DataPrintBool (LCD_MENU_Data* dataItem, uint8_t row) {
	LCD_SetCursor(dataItem->hlcd, LCD_DATA_COL, row);
	if (dataItem->value > 0) {
		LCD_Print(dataItem->hlcd, "ON ");
	}
	else {
		LCD_Print(dataItem->hlcd, "OFF");
	}
}

/**
 * Print three digits of the dataItem value at row
 * @param dataItem Pointer to the menu data
 * @param row The row to print to the LCD
 */
void LCD_MENU_DataPrint (LCD_MENU_Data* dataItem, uint8_t row) {
	char str_buff[21] = {'\0'};
	snprintf(str_buff, 21, "%03d", dataItem->value);
    LCD_SetCursor(dataItem->hlcd, LCD_DATA_COL, row);
    LCD_Print(dataItem->hlcd, str_buff);
}

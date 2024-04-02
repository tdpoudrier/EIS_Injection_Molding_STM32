/*
 * LCD_Menu.h
 *
 *  Created on: Apr 1, 2024
 *      Author: Tevin Poudrier
 */

#ifndef INC_LCD_MENU_H_
#define INC_LCD_MENU_H_

#include "LCD_I2C.h"
#include "stm32c0xx_hal.h"

#define MAX_MENU_ITEMS 10

typedef struct __LCD_MENU_Item {
	char * rowText[4];
	char * itemName;
} LCD_MENU_Item;

typedef struct __LCD_MENU_List {
	LCD_HandleTypeDef* hlcd;
	LCD_MENU_Item* items[MAX_MENU_ITEMS];
	int cursor;
	int numItems;
} LCD_MENU_List;

void LCD_MENU_ListInit (LCD_HandleTypeDef* hlcd, LCD_MENU_List* list, LCD_MENU_Item items[], int numItems);

void LCD_MENU_ItemInit (LCD_MENU_Item* item, char name[]);

void LCD_MENU_PrintList (LCD_MENU_List* list, int startIndex);

void LCD_MENU_MoveCursor (LCD_MENU_List* list, int index);

void LCD_MENU_PrintItem (LCD_MENU_List* list, int index);

#endif /* INC_LCD_MENU_H_ */

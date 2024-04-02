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
#define MENU_STRING_LENGTH 16

//TODO - Determine if one struct with a "linked-list" structure would work

typedef struct __LCD_MENU_Item {
	char * rowText[4];
	char * itemName;
} LCD_MENU_Item;

typedef struct __LCD_MENU_List {
	LCD_HandleTypeDef* hlcd;
	LCD_MENU_Item* items[MAX_MENU_ITEMS];
	int cursor;
	int printedIndex;
	int numItems;
} LCD_MENU_List;

void LCD_MENU_ListInit (LCD_HandleTypeDef* hlcd, LCD_MENU_List* list);

void LCD_MENU_AddItem (LCD_MENU_List* list, LCD_MENU_Item* item, char* name, char * text[]);

void LCD_MENU_PrintList (LCD_MENU_List* list, int startIndex);

void LCD_MENU_MoveCursor (LCD_MENU_List* list, int direction);

void LCD_MENU_PrintItem (LCD_MENU_List* list, int column);

#endif /* INC_LCD_MENU_H_ */

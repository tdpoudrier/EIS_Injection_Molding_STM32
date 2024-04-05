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

#define MENU_STRING_LENGTH 16

#define MV_CURSOR_DOWN 1
#define MV_CURSOR_UP 0

#define ITEM_TYPE_DISPLAY 0
#define ITEM_TYPE_CONFIG 1

//TODO - Determine if one struct with a "linked-list" structure would work

typedef struct __LCD_MENU_Item {
	char * rowText[4];
	char * itemName;
	int type;
	int cursor;
	int dataElement[4];
	struct __LCD_MENU_Item* parent;
	struct __LCD_MENU_Item* child;
} LCD_MENU_Item;

typedef struct __LCD_MENU_List {
	LCD_HandleTypeDef* hlcd;
	LCD_MENU_Item* items[4];
	int cursor;
	int numItems;
	struct __LCD_MENU_List* parent;
	struct __LCD_MENU_List* child;
} LCD_MENU_List;

/*
 * Menu List Functions
 */
void LCD_MENU_ListInit (LCD_HandleTypeDef* hlcd, LCD_MENU_List* list);

void LCD_MENU_ExtendList (LCD_MENU_List* parent, LCD_MENU_List* extension);

LCD_MENU_List* LCD_MENU_MoveListCursor (LCD_MENU_List* list, uint8_t direction);

void LCD_MENU_PrintList (LCD_MENU_List* list);

void LCD_MENU_AddItemToList (LCD_MENU_List* listHead, LCD_MENU_Item* item);

/*
 * Menu Item Functions
 */
void LCD_MENU_ItemInit (LCD_MENU_Item* item, char name[], char * text[], uint8_t type);

void LCD_MENU_ExtendItem(LCD_MENU_Item* parent, LCD_MENU_Item* extenstion);

void LCD_MENU_PrintItem (LCD_MENU_List* list);

#endif /* INC_LCD_MENU_H_ */

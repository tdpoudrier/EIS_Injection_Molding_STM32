/*
 * LCD_Menu.h
 *
 *  Created on: Apr 1, 2024
 *      Author: Tevin Poudrier
 *      Description: Driver for menu interface with 20x4 LCD. List allow selection of 4 menu items. Menu items display text and data. Cursor is used to select.
 *      	Actions are stored per LCD row on menu items.
 */

#ifndef INC_LCD_MENU_H_
#define INC_LCD_MENU_H_

#include "LCD_I2C.h"
#include "stm32F1xx_hal.h"
#include "LCD_Menu_Data.h"

#define MENU_STRING_LENGTH 16

#define MV_CURSOR_DOWN 1
#define MV_CURSOR_UP 0

#define ITEM_TYPE_DISPLAY 0
#define ITEM_TYPE_CONFIG 1

#define ITEM_ACTION_EMPTY 0
#define ITEM_ACTION_DATA 1
#define ITEM_ACTION_RETURN 2
#define ITEM_ACTION_TOGGLE 3
#define ITEM_ACTION_SELECT 4

/**
 * LCD_MENU_Item struct
 * used to display 4 rows of strings and data
 * Can be extended with another list to display more data
 * cursor represents the selected row and coresposnts to ITEM_ACTIONs stored in rowType
 */
typedef struct __LCD_MENU_Item {
	LCD_HandleTypeDef* hlcd;
	char * rowText[4];
	char * itemName;
	uint8_t type;
	int8_t cursor;
	uint8_t rowType[4];
	uint8_t dataElementRowPos[10];
	uint8_t dataElementColPos[10];
	LCD_MENU_Data* dataElement[10];
	struct __LCD_MENU_Item* parent;
	struct __LCD_MENU_Item* child;
} LCD_MENU_Item;

/**
 * LCD_MENU_List struct
 * Used to display the names of at most 4 menu items
 * cursor represents selected item
 * Can be extended with another list to show more items
 */
typedef struct __LCD_MENU_List {
	LCD_HandleTypeDef* hlcd;
	LCD_MENU_Item* items[4];
	int8_t cursor;
	uint8_t numItems;
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
void LCD_MENU_ItemInit (LCD_HandleTypeDef* hlcd, LCD_MENU_Item* item, char name[], char * text[], uint8_t type);

void LCD_MENU_ExtendItem(LCD_MENU_Item* parent, LCD_MENU_Item* extenstion);

void LCD_MENU_ItemAddData(LCD_MENU_Item* item, LCD_MENU_Data* dataItem, int8_t row, uint8_t ITEM_ACTION_XX);

LCD_MENU_Item* LCD_MENU_MoveItemCursor (LCD_MENU_Item* list, uint8_t direction);

void LCD_MENU_ItemSetAction (LCD_MENU_Item* item, uint8_t index, uint8_t action);

void LCD_MENU_PrintItem (LCD_MENU_Item* list);

void LCD_MENU_UpdateItemData(LCD_MENU_Item* item);

void LCD_MENU_ItemSetString (LCD_MENU_Item* item, uint8_t row, char* string);

#endif /* INC_LCD_MENU_H_ */

/*
 * LCD_Menu.c
 *
 *  Created on: Apr 1, 2024
 *      Author: Tevin Poudrier
 */

#include "LCD_Menu.h"
#include <string.h>

//TODO convert data times to embedded typedefs like uint8_t

/**
 * Initialize menu list
 */
void LCD_MENU_ListInit (LCD_HandleTypeDef* hlcd, LCD_MENU_List* list) {
	list->hlcd = hlcd;
	list->cursor = 0;
	list->numItems = 0;

	for(uint8_t i = 0; i < 4; i++) {
		list->items[i] = NULL;
	}

	list->parent = NULL;
	list->child = NULL;
}

void LCD_MENU_ExtendList (LCD_MENU_List* parent, LCD_MENU_List* extension) {
	parent->child = extension;
	extension->parent = parent;
}

/**
 * Increment the cursor on the list. if cursor is moved past list, print new list
 * returns pointer to current printed list
 */
LCD_MENU_List* LCD_MENU_MoveListCursor (LCD_MENU_List* list, uint8_t direction) {
	uint8_t prevCursor = list->cursor;

	if (direction == 1) {
		list->cursor++;
	}
	else {
		list->cursor--;
	}

	//List cursor has past the end of list, go to child list
	if(list->cursor > list->numItems - 1) {
		if(list->child != NULL) {
			list->cursor--;
			LCD_MENU_PrintList(list->child);

			return list->child;
		}
		else {
			list->cursor--;
		}
	}
	//List cursor has past the start of list, go to parent list
	else if(list->cursor < 0) {
		if(list->parent != NULL) {
			list->cursor = 0;
			LCD_MENU_PrintList(list->parent);

			return list->parent;
		}
		else {
			list->cursor++;
		}
	}

	//Print cursor
	LCD_SetCursor(list->hlcd, 0, prevCursor);
	LCD_PrintChar(list->hlcd, ' ');

	LCD_SetCursor(list->hlcd, 0, list->cursor);
	LCD_PrintChar(list->hlcd, '>');

	return list;
}

/**
 * Initialize menu item. Sets the name and text of menu item.
 * @param name is a string
 * @param text is an array of strings
 * text must have 4 strings
 */
void LCD_MENU_ItemInit (LCD_HandleTypeDef* hlcd, LCD_MENU_Item* item, char name[], char * text[], uint8_t type) {
	item->itemName = name;
	item->hlcd = hlcd;

	for(uint8_t i = 0; i < 4; i++) {

		item->rowText[i] = text[i];
		item->rowType[i] = ITEM_ACTION_EMPTY;
		item->dataElement[i] = NULL;
	}

	if(type <= 1) {
			item->type = type;
		}
	else {
		while(1);
	}

	/*
	 * Pointers are implicity set to NULL, left here for readability
	 * item->parent == NULL;
	 * item->child == NULL;
	*/
}

/**
 * Add menu item to the list. Sets the name and text of menu item.
 * @param name is a string
 * @param text is an array of strings
 * text must have 4 strings
 */
void LCD_MENU_AddItemToList (LCD_MENU_List* listHead, LCD_MENU_Item* item) {

	LCD_MENU_List* node = listHead;
	while(node->numItems == 4) {
		if(node->child != NULL) {
			node = node->child;
		}
		else {
			while(1);
		}
	}
	node->items[node->numItems] = item;
	node->numItems++;


}

/**
 * Increment the cursor on the list. if cursor is moved past list, print new list
 * returns pointer to current printed list
 */
void LCD_MENU_ExtendItem(LCD_MENU_Item* parent, LCD_MENU_Item* extenstion) {
	parent->child = extenstion;
	extenstion->parent = parent;
}

/**
 * Increment the cursor on the list. if cursor is moved past list, print new list
 * returns pointer to current printed list
 */
LCD_MENU_Item* LCD_MENU_MoveItemCursor (LCD_MENU_Item* item, uint8_t direction) {
	uint8_t prevCursor = item->cursor;

	if (item->type == ITEM_TYPE_DISPLAY) {
		return item;
	}
	//Calculate direction
	if (direction == 1) {
		item->cursor++;
	}
	else {
		item->cursor--;
	}

	//List cursor has past the end of list, go to child list
	if(item->cursor > 3) {
		if(item->child != NULL) {
			item->cursor--;
			LCD_MENU_PrintItem(item->child);

			return item->child;
		}
		else {
			item->cursor--;
		}
	}
	//List cursor has past the start of list, go to parent list
	else if(item->cursor < 0) {
		if(item->parent != NULL) {
			item->cursor = 0;
			LCD_MENU_PrintItem(item->parent);

			return item->parent;
		}
		else {
			item->cursor++;
		}
	}

	//Print cursor
	LCD_SetCursor(item->hlcd, 0, prevCursor);
	LCD_PrintChar(item->hlcd, ' ');

	LCD_SetCursor(item->hlcd, 0, item->cursor);
	LCD_PrintChar(item->hlcd, '>');

	return item;
}

/**
 * Define the action of a menu item row
 */
void LCD_MENU_ItemSetAction (LCD_MENU_Item* item, uint8_t index, uint8_t action) {
	if (action > 2 || index > 3) {
		return;
	}

	item->rowType[index] = action;
}

/**
 * Print the list and its cursor to the LCD
 */
void LCD_MENU_PrintList (LCD_MENU_List* list) {
	LCD_Clear(list->hlcd);

	//Print cursor
	LCD_SetCursor(list->hlcd, 0, list->cursor);
	LCD_PrintChar(list->hlcd, '>');

	//i represents item index being printed
	//j represents row to print data on
	for( uint8_t i = 0, j = 0; i < list->numItems && j < 4; i++, j++) {
		LCD_SetCursor(list->hlcd, 1, j);
		LCD_Print(list->hlcd, list->items[i]->itemName);
	}
}


/**
 * Print a menu item to the lcd based on the cursor location
 */
void LCD_MENU_PrintItem (LCD_MENU_Item* item) {

	LCD_Clear(item->hlcd);

	LCD_SetCursor(item->hlcd, 0, item->cursor);
	LCD_PrintChar(item->hlcd, '>');

	for(uint8_t i = 0; i < 4; i++) {
		LCD_SetCursor(item->hlcd, item->type, i);
		LCD_Print(item->hlcd, item->rowText[i]);

		if (item->dataElement[i] != NULL) {
			LCD_MENU_DataPrint(item->dataElement[i]);
		}
	}
}

/**
 * Add a data element to a menu item
 */
void LCD_MENU_ItemAddData(LCD_MENU_Item* item, LCD_MENU_Data* dataItem) {
	uint8_t index = dataItem->rowPos;
	item->dataElement[index] = dataItem;
	item->rowType[index] = ITEM_ACTION_DATA;
}


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

	for(int i = 0; i < 4; i++) {
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
	int prevCursor = list->cursor;

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
void LCD_MENU_ItemInit (LCD_MENU_Item* item, char name[], char * text[], uint8_t type) {
	item->itemName = name;

	for(int i = 0; i < 4; i++) {

		item->rowText[i] = text[i];
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

/*
 * Attach a menu item to another menu item, allowing for than 4 lines to be printed
 */
void LCD_MENU_ExtendItem(LCD_MENU_Item* parent, LCD_MENU_Item* extenstion) {
	parent->child = extenstion;
	extenstion->parent = parent;
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
	for( int i = 0, j = 0; i < list->numItems && j < 4; i++, j++) {
		LCD_SetCursor(list->hlcd, 1, j);
		LCD_Print(list->hlcd, list->items[i]->itemName);
	}
}


/**
 * Print a menu item to the lcd based on the cursor location
 */
void LCD_MENU_PrintItem (LCD_MENU_List* list) {
	LCD_MENU_Item* item = list->items[list->cursor];

	LCD_Clear(list->hlcd);

	for(int i = 0; i < 4; i++) {
		LCD_SetCursor(list->hlcd, item->type, i);
		LCD_Print(list->hlcd, item->rowText[i]);
	}
}


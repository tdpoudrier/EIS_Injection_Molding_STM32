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
	list->printedIndex = 0;
}

/**
 * Add menu item to the list. Sets the name and text of menu item.
 * @param name is a string
 * @param text is an array of strings
 * text must have 4 strings
 */
void LCD_MENU_AddItem (LCD_MENU_List* list, LCD_MENU_Item* item, char* name, char * text[]) {

	item->itemName = name;

	for(int i = 0; i < 4; i++) {

		item->rowText[i] = text[i];
	}

	list->items[list->numItems] = item;
	list->numItems++;

}

/**
 * Print 4 elements from the list to the LCD
 * @param startIndex specifies what elements in the list will be printed
 */
void LCD_MENU_PrintList (LCD_MENU_List* list, int startIndex) {
	if (startIndex > list->numItems) {
		while(1);
	}

	LCD_Clear(list->hlcd);
	list->printedIndex = startIndex;

	//i represents item index being printed
	//j represents row to print data on
	for( int i = startIndex, j = 0; i < list->numItems && j < 4; i++, j++) {
		if(i == list->cursor) {
			LCD_SetCursor(list->hlcd, 0, j);
			LCD_Print(list->hlcd, ">");
		}

		LCD_SetCursor(list->hlcd, 1, j);
		LCD_Print(list->hlcd, list->items[i]->itemName);
	}
}

//TODO - Change to SetCursor which does not print to LCD, that will be left to application
/**
 * Move the cursor up or down, reprint list if needed
 * @param direction defines move direction, up when -1 and down when 1
 */
void LCD_MENU_MoveCursor (LCD_MENU_List* list, int direction) {
	int prevCursor = list->cursor;
	list->cursor += direction;

	//list wraps around
	if(list->cursor < 0) {
		list->cursor = list->numItems;
		if (list->numItems < 3) {
			LCD_MENU_PrintList (list, 0);
			list->cursor = 0;
		}
		else {
			list->cursor = list->numItems - 3;
			LCD_MENU_PrintList (list, list->numItems - 3);
		}

	}

	else if(list->cursor > list->numItems-1) {
		list->cursor = 0;
		LCD_MENU_PrintList (list, 0);
	}

	else if(list->cursor - list->printedIndex > 3 || list->cursor - list->printedIndex < -3) {
		LCD_MENU_PrintList (list, list->cursor - 3);
	}

	else {
		LCD_SetCursor(list->hlcd, 0, prevCursor);
		LCD_Print(list->hlcd, " ");

		LCD_SetCursor(list->hlcd, 0, list->cursor);
		LCD_Print(list->hlcd, ">");
	}




}


/**
 * Print a menu item to the lcd at specifed starting column
 * @param column intended to provide room for the cursor if needed
 * //column is for print location on LCD, intended to provide room for cursor to select items.
 */
void LCD_MENU_PrintItem (LCD_MENU_List* list, int column) {
	LCD_MENU_Item* item = list->items[list->cursor];

	LCD_Clear(list->hlcd);

	for(int i = 0; i < 4; i++) {
		LCD_SetCursor(list->hlcd, column, i);
		LCD_Print(list->hlcd, item->rowText[i]);
	}
}


/*
 * LCD_Menu.c
 *
 *  Created on: Apr 1, 2024
 *      Author: Tevin Poudrier
 */

#include "LCD_Menu.h"
#include <string.h>

void LCD_MENU_ListInit (LCD_HandleTypeDef* hlcd, LCD_MENU_List* list, LCD_MENU_Item items[], int numItems) {
	list->hlcd = hlcd;
	list->cursor = 0;
	list->numItems = numItems;

	for (int i = 0; i < numItems; i++) {
		list->items[i] = &items[i];
	}


}

void LCD_MENU_ItemInit (LCD_MENU_Item* item, char* name) {
	item->itemName = name;
}

void LCD_MENU_PrintList (LCD_MENU_List* list, int startIndex) {
	if (startIndex > list->numItems) {
		while(1);
	}

	LCD_Clear(list->hlcd);

	//i represents item index being printed
	//j represents row to print data on
	for( int i = startIndex, j = 0; i < list->numItems && j < 4; i++, j++) {
		LCD_SetCursor(list->hlcd, 0, j);
		LCD_Print(list->hlcd, list->items[i]->itemName);
	}
}

void LCD_MENU_MoveCursor (LCD_MENU_List* list, int index);

void LCD_MENU_PrintItem (LCD_MENU_List* list, int index);


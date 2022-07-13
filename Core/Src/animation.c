/*
 * animation.c
 *
 *  Created on: 7 lug 2022
 *      Author: Mattia
 */

#include "st_logo.h"
#include "ssd1306.h"


void anim ()
{
	int i=0;

	for (int i=0;i<50;i++)
	{

	SSD1306_DrawBitmap(i, 0, logo, 128, 64, 1);
    SSD1306_UpdateScreen();

	HAL_Delay(200);
	SSD1306_Clear();
	SSD1306_DrawBitmap(i, 0, logo, 128, 64, 1);
	SSD1306_UpdateScreen();

	}
}

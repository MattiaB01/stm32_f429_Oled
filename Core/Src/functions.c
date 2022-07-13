/*
 * function.c
 *
 *  Created on: 13 lug 2022
 *      Author: Mattia
 */

#include "ssd1306.h"
#include <stdio.h>
#include "functions.h"

int32_t readValue;
char c[4];

int32_t offset=10;


void displayAdc()
{
readValue=read_conv();


int posX=14;
if (readValue<10)
{
	posX*=4;
	sprintf(c,"   %d",readValue);
}
else if (readValue < 100)
{
	posX*=3;
	sprintf(c,"  %d",readValue);
}
else if (readValue <1000)
{
	posX*=2;
	sprintf(c," %d",readValue);
}
else
{

	sprintf(c,"    %d",readValue);
}


SSD1306_GotoXY(posX, 40);
SSD1306_Puts(c, &Font_11x18, 1);
SSD1306_UpdateScreen(); //display
HAL_Delay(100);
}

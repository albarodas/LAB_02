/* 
 * File: LCD.c  
 * Se utilizó y se adaptaron las librerías de Ligo George 
 * de la página www.electrosome.com
 * Enlace: https://electrosome.com/lcd-pic-mplab-xc8/
 * Revision history: 
 */


#include <xc.h>
#include "LCD.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <pic16f887.h>

#define _XTAL_FREQ 8000000

void Lcd_Port(char a)
{
	PORTD = a;
}
void Lcd_Cmd(char a)
{
	RS = 0;             // => RS = 0
	Lcd_Port(a);
	EN  = 1;             // => E = 1
    __delay_ms(4);
    EN  = 0;             // => E = 0
}

void Lcd_Clear()
{
	Lcd_Cmd(0);
	Lcd_Cmd(1);
}

void Lcd_Set_Cursor(char a, char b)
{
	char c;
	if(a == 1)
	{
		c = 0x80 + b -1;
		Lcd_Cmd(c);
	}
	else if(a == 2)
	{
		c = 0xC0 + b - 1;
		Lcd_Cmd(c);
	}
}

void Lcd_Init()
{
  Lcd_Port(0x00);
  __delay_ms(20);
  Lcd_Cmd(0x30);
  __delay_ms(5);
  Lcd_Cmd(0x30);
  __delay_ms(11);
  Lcd_Cmd(0x30);
  /////////////////////////////////////////////////////
  Lcd_Cmd(0x38);
  Lcd_Cmd(0x0C);
  Lcd_Cmd(0x01);
  Lcd_Cmd(0x06);
  __delay_ms(100);
}


void Lcd_Write_Char(char a)
{
   RS = 1;            
   Lcd_Port(a);            
   EN = 1;
   __delay_ms(4);
   EN = 0;
}

void Lcd_Write_String(char *a)
{
	int i;
	for(i=0;a[i]!='\0';i++)
	   Lcd_Write_Char(a[i]);
}

void Lcd_Shift_Right()
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left()
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x08);
}
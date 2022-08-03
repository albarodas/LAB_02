/* 
 * File:   TMR0.c
 * Author: ALBA RODAS
 *
 * Created on 22 de julio de 2022, 08:20 AM
 */
//-----------------------------------------------------------------------------
// IMPORTAMOS LIBRERIAS:
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "TMR0.h"
//---------------------------------------------------------------
void TMR0_init(uint8_t prescaler_VALUE){
    OPTION_REGbits.T0CS = 0;
    if (prescaler_VALUE == 0)
        OPTION_REGbits.PSA = 1; //Se asigno el prescaler_VALUE para el TMR0
    else
        OPTION_REGbits.PSA = 0; //Se asigno el prescaler_VALUE para el TMR0
    switch (prescaler_VALUE)
    {
        case 2: // VALOR DE PRESCALER: --> 000
            OPTION_REGbits.PS0 = 0;
            OPTION_REGbits.PS1 = 0;
            OPTION_REGbits.PS2 = 0;
            break;
            
        case 4: // VALOR DE PRESCALER: --> 100
            OPTION_REGbits.PS0 = 1;
            OPTION_REGbits.PS1 = 0;
            OPTION_REGbits.PS2 = 0;
            break;
            
        case 8: // VALOR DE PRESCALER: --> 010
            OPTION_REGbits.PS0 = 0;
            OPTION_REGbits.PS1 = 1;
            OPTION_REGbits.PS2 = 0;
            break;
            
        case 16: // VALOR DE PRESCALER: --> 110
            OPTION_REGbits.PS0 = 1;
            OPTION_REGbits.PS1 = 1;
            OPTION_REGbits.PS2 = 0;
            break;
            
        case 32: // VALOR DE PRESCALER: --> 001
            OPTION_REGbits.PS0 = 0;
            OPTION_REGbits.PS1 = 0;
            OPTION_REGbits.PS2 = 1;
            break;
            
        case 64: // VALOR DE PRESCALER: --> 101
            OPTION_REGbits.PS0 = 1;
            OPTION_REGbits.PS1 = 0;
            OPTION_REGbits.PS2 = 1;
            break;
            
        case 128: // VALOR DE PRESCALER: --> 011
            OPTION_REGbits.PS0 = 0;
            OPTION_REGbits.PS1 = 1;
            OPTION_REGbits.PS2 = 1;
            break;
            
        case 255: // VALOR DE PRESCALER: --> 111
            OPTION_REGbits.PS0 = 1;
            OPTION_REGbits.PS1 = 1;
            OPTION_REGbits.PS2 = 1;
            break;
    }
    TMR0 = VALOR_CARGA;
    INTCONbits.T0IE = 1;        // INTERRUPCIÓN DEL TMR = ON.
    INTCONbits.T0IF = 0;        
}

void TMR0_CARGA(void)
{
    TMR0 = VALOR_CARGA;     // TMR0 = VALOR_CARGA
    INTCONbits.T0IF = 0;    
}
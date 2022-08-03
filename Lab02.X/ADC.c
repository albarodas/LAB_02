/* 
 * File:   ADC.c
 * Author: ALBA RODAS
 *
 * Created on 19 de julio de 2022, 06:54 AM
 */
// DEFINIMOS UN VALOR DE OSCILADOR:
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 4000000
#endif
/*===============================================================================
 * INCLUIMOS LAS LIBRERIAS ".h" A UTILIZAR:
================================================================================*/
 #include <xc.h>
#include <stdint.h>
#include "ADC.h"
 
void initADC(uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_min){
    //ADCON0bits.ADCS = adc_cs;
    switch(adc_cs){
        case 0: // --> FOSC/2
            ADCON0bits.ADCS = 0b00;
            break;
        case 1: // --> FOSC/8
            ADCON0bits.ADCS = 0b01;
            break;
        case 2: // --> FOSC/32
            ADCON0bits.ADCS = 0b10;
            break;
        default: // --> FOSC PARA EL RFC
            ADCON0bits.ADCS = 0b11;
            break;
    // AQUI CONFIGURAMOS EL ADC_CS: 
    //00 = FOSC/2
    //01 = FOSC/8
    //10 = FOSC/32
    //11 = Frc
   //ADCON0bits.CHS = 0b000;
   /* ADCON1bits.ADFM = 1;
    ADCON0bits.ADON = 1;
    __delay_ms(1);*/
    }
    ADCON1bits.ADFM = 0;
    ADCON1bits.VCFG0 = vref_plus;
    ADCON1bits.VCFG1 = vref_min;
    ADCON0bits.ADON =  1;
    __delay_ms(1);
    
    INTCONbits.PEIE = 1;        
    PIE1bits.ADIE = 1;          
    PIR1bits.ADIF = 0;          
}

void adc_start(uint8_t channel_1, uint8_t channel_2){
        if (ADCON0bits.GO == 0) 
        {                           
            if (ADCON0bits.CHS == channel_1)
            {                       
                __delay_us(50);                            
                ADCON0bits.CHS = channel_2;                         
            }                                               
            else if (ADCON0bits.CHS == channel_2)
            {                  
                __delay_us(70);                            
                ADCON0bits.CHS = channel_1;                         
            }                                                                                         
            __delay_us(100);                                
            ADCON0bits.GO = 1;                              
        }
}
uint16_t read_ADC(void)
{
    PIR1bits.ADIF = 0;                  
}

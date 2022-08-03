/* 
 * File:   ADC.h
 * Author: ALBA RODAS
 *
 * Created on 22 de julio de 2022, 08:20 AM
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef ADC_H
#define	ADC_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

    // AQUI CONFIGURAMOS EL ADC_CS: 
    //00 = FOSC/2
    //01 = FOSC/8
    //10 = FOSC/32
    //11 = Frc
   //ADCON0bits.CHS = 0b000;
   /* ADCON1bits.ADFM = 1;
    ADCON0bits.ADON = 1;
    __delay_ms(1);*/

void initADC(uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_min);
void adc_start(uint8_t channel_1, uint8_t channel_2);
uint16_t read_ADC(void);

#endif	/* XC_HEADER_TEMPLATE_H */


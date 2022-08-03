/* 
 * File:   oscilador.h
 * Author: ALBA RODAS
 *
 * Created on 22 de julio de 2022, 08:20 AM
 */
//------------------------------------------------------------------------------
// LIBRERIAS POR DEFAULT: 
#ifndef OSCILADOR_H
#define	OSCILADOR_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h> 

void initOsc(uint8_t freq);
/*{
 * // PARA SABER QUE ESTOY PROGRAMANDO.
    OSCCONbits.IRCF = freq;
    switch(freq){
        // VALOR: 1MHz --> 100
        case 0:             
            OSCCONbits.IRCF = 0b100;
            OSCCONbits.SCS = 1;
            break;
        // VALOR: 2MHz --> 101
        case 1:
            OSCCONbits.IRCF = 0b101;
            OSCCONbits.SCS = 1;
            break;
       // VALOR: 4MHz --> 110
       case 2:
            OSCCONbits.IRCF = 0b110;
            OSCCONbits.SCS = 1;
            break;
       // VALOR: 8MHz --> 111
       default:
            OSCCONbits.IRCF = 0b111;
            OSCCONbits.SCS = 1;
            break;       
    }
}
*/
#endif	/* XC_HEADER_TEMPLATE_H */


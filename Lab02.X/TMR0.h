/* 
 * File:   TMR0.h
 * Author: ALBA RODAS
 *
 * Created on 19 de julio de 2022, 06:36 AM
 */
// --------------------------------------------------------------
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef TMR0_H
#define	TMR0_H
// VALOR TOMADO DE LABS ANTERIORES PARA 500ms
#define VALOR_CARGA 61  

#include <xc.h> 
// include processor files - each processor file is guarded.  
#include <stdint.h>
void TMR0_init(uint8_t prescaler_VALUE);
// CON ESTA DEFINIMOS FUNCION PARA CARGAR PRESCALER
void TMR0_CARGA(void);
#endif	/* XC_HEADER_TEMPLATE_H */


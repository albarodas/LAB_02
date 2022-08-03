/* 
 * File:   LAB02_MAIN.c
 * Author: ALBA RODAS.
 * Created on 19 de julio de 2022, 06:36 AM
 */
// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
/*===============================================================================
 * INCLUIMOS LAS LIBRERIAS ".h" A UTILIZAR:
================================================================================*/
#include <xc.h>
#include "TMR0.h"
#include "LCD.h"
#include "Oscilador.h"
#include "ADC.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

//-------------------------------------------------------------------------------
// DECLARAMOS FREQ. DE OSCILADOR:
#define _XTAL_FREQ 1000000      // VALOR DE OSC: 1MHz
//-------------------------------------------------------------------------------
// CONFIGURATION WORDS: 1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIGURATION WORDS: 2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
//-------------------------------------------------------------------------------
//===============================================================================
// VARIABLES:
//===============================================================================
//-------------------------------------------------------------------------------
// VARIABLES PARA POTENCIOMETROS:
char POT_01;                    // POTENCIOMETROS --> ADC
char POT_02;                    // P1 = POTENCIOMETRO #1, P2 = POTENCIOMETRO #2.
//------------------------------------------------------------------------------
// VARIABLES PARA EL MANEJO DE DECIMAES/UNIDADES DEL LCD:
char unidades;                  // DEFINIMOS UNIDADES Y DECENAS PARA TAMBIEN MANEJAR LAS 3 CIFAS SIG.
char decenas;
char centenas;
//------------------------------------------------------------------------------
// VARIABLE PARA CONTADORES:
uint8_t counter_tmr0;           // VARIABLE PARA CONTADOR DEL TMR0
uint8_t counter_time;           // VARIABLE PARA CONTADOR DE LA LCD.
//------------------------------------------------------------------------------
// VARIABLE PARA MAPEO DE VOLTAJE:
float VOTAJE_01 = 0;
float VOTAJE_02 = 0;
//------------------------------------------------------------------------------
// VARIABLES PARA MAPEO DE VOLTAJE (LISTADO):
char VOTAJE_L01[5];
char VOLTAJE_L02[5];
//------------------------------------------------------------------------------
// VARIABLES PARA CAMBIO DE SHOW VOLTAJE (POT):
uint8_t change_POT = 0;         // BANDERA -> FLAG ADC.
/*===============================================================================
 * PROTOTIPO DE FUNCIONES:
================================================================================*/
void setup(void);
/*===============================================================================
 * INTERRUPCIONES:
================================================================================*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0)
            POT_01 = ADRESH;        // GUARDAMOS VALOR DE LOS POT. EN EL ADRESH.
        else
            POT_02 =  ADRESH;
        read_ADC();
    }
//--------------------------------------------------------------------------
// PARA DAR LOS 5 SEG. DE CAMBIO ENTRE P1 y P2 DAMOS 25 VUELTAS:
//--------------------------------------------------------------------------
    if(INTCONbits.T0IF){
        counter_tmr0++;             // INCREMENTAMOS LE CONTADOR DEL TMR0.
        if(counter_tmr0 == 25)
        {       
            counter_tmr0 = 0;
            if(change_POT == 0)
            {
                change_POT = 1;  
            }
            else
            {
                change_POT = 0;
            }
        }
//--------------------------------------------------------------------------
        TMR0_CARGA();              // --> TMR0.h, TMR0.c PARA MODIFICACIONES.
//--------------------------------------------------------------------------
    }
    return;
}
/*===============================================================================
 * CONFIGURACION DE ENTRADAS Y SALIDAS:
================================================================================*/
void setup(void){
    // HABILITAMOS PINES ANALOGICOS, SI HUBIERAN:
    ANSELH = 0;               // --> PINES DIGITALES
    ANSEL = 0b011;       // --> ANALOGICOS PARA POTENCIOMETROS --> A0, A1.
//****************************************************************
// DEFINIMOS SALIDAS:    
    TRISA = 0b011;       // SALIDA DEL CONTADOR --> NO DEL TMR0.
    TRISB = 0;                
    TRISC = 0;
    TRISD = 0;                
    TRISE = 0;                
//****************************************************************
//--------------------------------------------------------------------------
// CLR TODOS LOS PUERTOS:    
    PORTA = 0;                
    PORTB = 0;                
    PORTC = 0;                // IN CASE LO NECESITE, YA ESTÁ ACTIVO.
    PORTD = 0;                
    PORTE = 0;                
//****************************************************************   
//--------------------------------------------------------------------------
    // INTERRUPCIONES --> PARA UTILIZAR LAS INTERRUPCIONES DEL TMR0, SE ACTIVAN LAS GLOBALES:
    INTCONbits.GIE = 1;
    // CONFIG. RELOJ INTERNO PIC.
    initOsc(0);               // OSCILADOR --> 1MHz.
// FRECUENCIA --> 1MHZ --> MEJOR FUNCIONAMIENTO DE LA LCD. 
//--------------------------------------------------------------------------
    // DEFINIMOS UN PRESCALER DE 1:256
    TMR0_init(255);  
    // DEFINIMOS UN ADC: 1) FOSC/2, VOLTAJE DE REFERENCIA = 0.
    initADC(0,0,0);          
}
/*===============================================================================
 * CONFIG. ADICIONALES: DETERMINANDO UNIDADES, DECENAS, CENTENAS --> LCD.
================================================================================*/
void convertion (void)
{
    // DEFINIMOS LAS FUNCIONES PARA UNIDADES, ETC:
    centenas = (counter_tmr0/100);              // DETERMINAMOS CENTENAS = VALOR CONTADOR/100.
    decenas = ((counter_tmr0-(centenas*100))/10);   // DETERMINAMOS DECENAS = VALOR CONTADOR - VALOR CONTADOR/100
    unidades = (counter_tmr0-(centenas*100 + decenas*10));
    // DETERMINAMOS UNIDADES = VALOR CONTADOR/100
    return;
}
/*===============================================================================
 * CICLO PRINCIPAL:
================================================================================*/
int main(void) 
{ 
    // "MENCIONAMOS" LA FUNCION DE CONFIGURACIÓN DE ENTRADAS Y SALIDAS:
    setup();    
    // INCLUIMOS LIBRERIA/FUNCION DE INICIALIZACIÓN DE LCD:
    Lcd_Clear();
    // LIMPIAMOS Y LUEGO INICIALIZAMOS:
    Lcd_Init();
    while(1)
    {
        // DEFINIMOS CONFIGURACIÓN DE ADC --> BASADO EN LIBRERIA.
        adc_start(0,1);
        // DEFINIMOS "ESCALA";
        VOTAJE_01 = (POT_01*0.01961);
        VOTAJE_02 = (POT_02*0.01961);
        // SI EL VALOR DE LA BANDERA DEL POT = 0...
        if(change_POT == 0)
        {
            // UTILIZAMOS COMANDOS DE LAS LIBRERIAS DADAS DEL LDC:
            // DEFINIMOS POSICIÓN DE CURSOR:
            Lcd_Set_Cursor(1,1);
            // DEFINIMOS ENCABEZADO: P1 --> POTENCIOMETRO #1.
            Lcd_Write_String("P1:");
            // FILA NO.2, COLUMINA 1.
            Lcd_Set_Cursor(2,1); 
            // MOSTRAMOS VALOR EN STRING:
            sprintf(VOTAJE_L01,"%.2fV", VOTAJE_01); 
            // MOSTRAMOS VALOR ACTUALIZADO DE VOLTAJE:
            Lcd_Write_String(VOTAJE_L01);              
        }
        else
        {
            // UTILIZAMOS COMANDOS DE LAS LIBRERIAS DADAS DEL LDC:
            // DEFINIMOS POSICIÓN DE CURSOR:
            Lcd_Set_Cursor(1,1);
            // DEFINIMOS ENCABEZADO: P2 --> POTENCIOMETRO #2.
            Lcd_Write_String("P2:");
            // FILA NO.2, COLUMINA 1.
            Lcd_Set_Cursor(2,1);     
            // MOSTRAMOS VALOR EN STRING:
            sprintf(VOTAJE_L01," %.2fV", VOTAJE_02);
            // MOSTRAMOS VALOR ACTUALIZADO DE VOLTAJE:
            Lcd_Write_String(VOTAJE_L01);
        }
    }
    return(1);
}
// ------------------------------FIN DEL CODIGO------------------------------
// ==========================================================================

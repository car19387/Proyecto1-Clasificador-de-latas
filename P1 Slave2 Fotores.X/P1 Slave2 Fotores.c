//******************************************************************************
//  Encabezado
//******************************************************************************
/* File: P1 Slave2 Fotores.c
 * Device: PIC16F887
 * Author: Angel Cuellar & Jefry Carrasco
 * Descripción: Fotoresistencia para medir la intesidad de la luz. PWM para 
 * controlar la intensidad de un sistema de luces. Comunicación I2C para envio 
 * de datos.
 * Creado: 17 de agosto, 2021
 * Modificado:  de agosto, 2021 */

//******************************************************************************
// Librerías incluidas
//******************************************************************************
#include <xc.h>
#include <stdint.h>
#include <proc/pic16f887.h>
#include "I2C.h"

//******************************************************************************
// Configuración de PIC16f887
//******************************************************************************

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

//******************************************************************************
// Directivas del preprocesador
//******************************************************************************
#define _XTAL_FREQ 8000000      // Para delay

//******************************************************************************
// Variables
//******************************************************************************
uint8_t Foto;                   // Almacenamiento de ADC
uint8_t z;                      // Envio de datos

//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void setup(void);               // Configuración

//******************************************************************************
// Main
//******************************************************************************
void main(void) {
    setup();                    // Iniciar configuraciones
    ADCON0bits.GO = 1;          // Iniciar conversion ADC
    
    //**************************************************************************
    // Loop principal
    //**************************************************************************
    while(1){
        if(ADCON0bits.GO == 0){         // Si la bandera del ADC se bajó
            if (ADCON0bits.CHS == 0){   // Si está en el canal 0
                ADCON0bits.CHS = 1;     // Entonces pasa al canal 1
            }  
            else {                      // Si no está en el canal 0
                ADCON0bits.CHS = 0;     // Entonces pasa al canal 0
            } 
            __delay_us(50);             //Delay para sample and hold
            ADCON0bits.GO = 1;          //Levantar bandera para conversi?n ADC
        }
    }
    return;
}

//******************************************************************************
// Interrupciones
//******************************************************************************
void __interrupt()isr(void){
    if(PIR1bits.SSPIF == 1){            
        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                 // Leer valor del buffer y limpiarlo
            SSPCONbits.SSPOV = 0;       // Se limpia la bandera de overflow
            SSPCONbits.WCOL = 0;        // Se limpia el bit de colision
            SSPCONbits.CKP = 1;         // Se habilita SCL
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            z = SSPBUF;                 // Leer valor del buffer y limpiarlo
            PIR1bits.SSPIF = 0;         // Limpia la bandera de SSP
            SSPCONbits.CKP = 1;         // Habilita los pulsos del reloj SCL
            while(!SSPSTATbits.BF);     // Esperar que la recepcion se realice
            __delay_us(250);   
        }
        
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;                 // Leer valor del buffer y limpiarlo
            BF = 0;
            SSPBUF = Foto;              // Escribe valor de la variable al buffer
            SSPCONbits.CKP = 1;         // Habilita los pulsos del reloj SCL
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
        PIR1bits.SSPIF = 0;    
    }
    
    if (PIR1bits.ADIF) {                // Interrupción del ADC
        if(ADCON0bits.CHS == 0) {       // Si está en el canal 0
            Foto = ADRESH;              // Guardar valor de conversión
            Foto = (Foto);              // Conversion del valor
            CCPR1L = Foto;              // Se despliegan en el CCP1
        }
        else {                          // Si está en el canal 1
            PORTB = ADRESH;             // Guardar valor de conversión en PORTB
            CCPR2L = (PORTB);           // Se despliega en el CCP2
        }
    PIR1bits.ADIF = 0;                  // Limpiar la bandera de ADC
    }
}

//******************************************************************************
// Configuraciones
//******************************************************************************
void setup() {
    // Configuración de Puertos
    ANSEL = 0x03;               // RA0 y RA1 como entrada analógica
    ANSELH = 0x00;
    
    TRISA = 0x03;               // Fotoresistencua en RA0
    TRISB = 0x00;
    TRISC = 0x08;               // Señal de reloj
    TRISD = 0x00; 
    TRISE = 0x00; 
    
    PORTA = 0x00;               // Limpiar puertos
    PORTB = 0x00;
    PORTC = 0x00;    
    PORTD = 0x00;
    PORTE = 0x00;
    
    // Configuracion del oscilador
    OSCCONbits.IRCF = 0b111;    // Frecuencia a 8MHZ
    OSCCONbits.SCS = 1;         // Habilitar reloj interno
    
    //Configurar ADC
    ADCON1bits.ADFM = 0;        // Justificar a la izquierda
    ADCON1bits.VCFG0 = 0;       // Voltaje de referencia Vss y Vdd
    ADCON1bits.VCFG1 = 0;       

    ADCON0bits.ADCS = 2;        // ADC clokc Fosc/32
    ADCON0bits.CHS = 0;         // Canal 0 selecionado     
    ADCON0bits.ADON = 1;        // Enecender módulo ADC
    __delay_us(50); 
    
    // Configurar TMR2
    CCP1CONbits.P1M = 0;        // PWM single output
    CCP1CONbits.CCP1M = 0b1100; // Se selecciona el modo PWM de CCP1  
    CCP2CONbits.CCP2M = 0b1100; // Se selecciona el modo PWM de CCP2
    
    CCPR1L = 0x0F;              // Valor inicial de CCPR1L
    CCPR2L = 0x0F;              // Valor inicial de CCPR2L
    CCP1CONbits.DC1B = 0;       // Bits menos significativos del Duty Cycle
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
    
    T2CONbits.T2CKPS1 = 1;      // Prescaler de 16
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1;       // Se enciende el TMR2
    PR2 = 250;                  // Valor inicial de PR2
    PIR1bits.TMR2IF = 0;        // Se limpia la bandera
    
    while (!PIR1bits.TMR2IF);   // Se espera una interrupci?n
    PIR1bits.TMR2IF = 0;        // Se limpia la bandera
  
    // Configuracion de interrupciones
    INTCONbits.GIE = 1;         // Se habilitan las interrupciones globales
    INTCONbits.PEIE = 1;        // Se habilitan las interrupciones perifericas
    PIE1bits.ADIE = 1;          // Se habilita la interrupcion del ADC
    PIR1bits.ADIF = 0;          // Se limpia la bandera del ADC
    I2C_Slave_Init(0x70);       // Direccion del esclavo  
}
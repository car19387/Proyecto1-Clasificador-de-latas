//******************************************************************************
//  Encabezado
//******************************************************************************
/* File: P1 Slave3 HC-SR04.c
 * Device: PIC16F887
 * Author: Angel Cuellar & Jefry Carrasco
 * Descripción: Sensor ultrasonico HC-SR04 para medir nivel de tanque. Alarma
 * para identificar cuando el tanque está vacio
 * Creado: 17 de agosto, 2021
 * Modificado:  de agosto, 2021 */

//******************************************************************************
// Librerías incluidas
//******************************************************************************
#include <xc.h>
#include <stdint.h>
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
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

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
uint16_t duracion;              // Duración del pulso                     
uint8_t distancia;              // Distancia en cm
uint8_t tnk;                    // Nivel del tanque
uint8_t z;                      // Envio de datos

//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void pulseOut(void);            // Función para enviar pulso ultrasonico
void pulseIn(void);             // Función para medir pulso
void setup(void);               // Configuración

//******************************************************************************
// Main
//******************************************************************************
void main(void) {
    setup();                    // Cinfiguración
    TMR1 = 0;                   // Limpiar el registro TMR1
    
    //**************************************************************************
    // Loop principal
    //**************************************************************************
    while(1){
        pulseOut();                 // Función para enviar pulso ultrasonico
        pulseIn();                  // Función para medir el tiempo de señal
        duracion = TMR1;            // Guardadr el tiempo del pulso
        distancia = (duracion)/58;  // Conversion de tiempo a distancia    
        
        if(distancia >= 40 && distancia <= 44){
            distancia = 40;         // Se restringe maximo a 40
            PORTBbits.RB2 = 1;      // Encender alarma
            __delay_ms(500);        // Tiempo encendido
            PORTBbits.RB2 = 0;      // Apagar alarma
            __delay_ms(750);        // Tiempo apagada
        }
        tnk = 100-((distancia-3)*2.702);    // Conversión para nivel de tanque
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
            SSPBUF = tnk;               // Escribe valor de la variable al buffer
            SSPCONbits.CKP = 1;         // Habilita los pulsos del reloj SCL
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
        PIR1bits.SSPIF = 0;    
    }
}

//******************************************************************************
// Configuraciones
//******************************************************************************
void setup(void){
    // Configuración de Puertos
    ANSELH = 0x00;              //Pines digitales
    ANSEL = 0x00;   
    
    TRISA = 0x00;               // Puertos como salida
    TRISB = 0x02;               // Trigger & Echo
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
    
    // Configurar TMR1
    T1CONbits.TMR1GE = 0; 
    T1CONbits.TMR1CS = 0; 
    T1CONbits.T1CKPS = 0b01;    // para pre_escaler = 1:2
    
    //Configurar interrupciones
    INTCONbits.GIE = 1;         // Se habilitan las interrupciones globales
    INTCONbits.PEIE = 1;        // Se habilitan las interrupciones perifericas
    I2C_Slave_Init(0x80);       // Direccion del esclavo                        
}

void pulseOut(void){
    RB0 = 1;                    // Encender el Trigger
    __delay_us(15);             // Pulso de 15 microsegundos
    RB0 = 0;                    // Se apaga el pulso
    TMR1 = 0;                   // Se carga valor al TMR1
}

void pulseIn(void){
    while (!RB1);               // Mientras el Echo este en bajo espera
    T1CONbits.TMR1ON = 1;       // Cuando Echo esta encendido inicia TMR1
    while(RB1);                 // Mientras este encendido cuenta
    T1CONbits.TMR1ON = 0;       // Cuando se apagague el pulso se apaga el TMR1
}
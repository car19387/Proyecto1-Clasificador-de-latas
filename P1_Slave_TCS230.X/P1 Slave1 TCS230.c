//******************************************************************************
//  Encabezado
//******************************************************************************
/* File: P1 Slave1 TCS230.c
 * Device: PIC16F887
 * Author: Angel Cuellar & Jefry Carrasco
 * Descripción: Sensor de color TCS230 para identificar gaseosas, control de 
 * servos para clasificación. Comunicación I2C para envio de datos.
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
uint16_t R;                      // Señal R
uint16_t V;                      // Señal G
uint16_t A;                      // Señal V
uint8_t cc = 0;                 // Cantidad de Coca Cola
uint8_t spr = 0;                // Cantidad de Sprite
uint8_t flag;                   // Bandera para lectura de color
uint8_t b;                      // Bandera para seleccion de dato
uint8_t z;                      // Envio de datos

//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void setup(void);               // Configuración
void pulseIn(void);             // Función para medir pulso
void getRojo(void);             // Obtener señal R
void getVerde(void);            // Obtener señal G
void getAzul(void);             // Obtener señal V

//******************************************************************************
// Main
//******************************************************************************
void main(void) {
    setup();                    // Iniciar configuraciones
    PORTBbits.RB1 = 1;          // Output frecuency 100%
    PORTBbits.RB2 = 1;
    
    //**************************************************************************
    // Loop principal
    //**************************************************************************
    while(1){
        PORTA = 240;             // Guardar valor de conversión en PORTB
        CCPR1L = (PORTA>>1) + 128;  // Valores válidos entre 128 y 250
            
        PORTA = 0;             // Guardar valor de conversión en PORTB
        CCPR2L = (PORTA>>1) + 128;  // Valores válidos entre 128 y 250

        getRojo();
        R = TMR1; 
        getVerde();
        V = TMR1;
        getAzul();
        A = TMR1;

        if (R >= 50 && R <= 70 && V < 90 && flag == 0) {
            cc = cc+1;
            flag = 1;
            PORTA = 100;             // Guardar valor de conversión en PORTB
            CCPR1L = (PORTA>>1) + 128;  // Valores válidos entre 128 y 250
            __delay_ms(15);
        }
        else if (R >= 71 && R <= 90 && V < 90 && flag == 0) {
            spr = spr+1;
            flag = 1;
            PORTA = 130;                // Guardar valor de conversión en PORTB
            CCPR2L = (PORTA>>1) + 128;  // Valores válidos entre 128 y 250
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
            b = SSPBUF;                 // Guardar el valor del buffer
            __delay_us(250);      
        }
        
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;                 // Leer valor del buffer y limpiarlo
            BF = 0;
            if(b == 0){                 // Si se pide el primer dato
                SSPBUF = cc;            // Escribe valor de la variable al buffer
            }
            if(b == 1){                 // Si se pide el segundo dato
                SSPBUF = spr;           // Escribe valor de la variable al buffer
            }
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
    ANSEL = 0x00;               // Pines digitales
    ANSELH = 0x00;
    
    /* out RB0
     * s0  RB1
     * s1  RB2
     * s2  RB3
     * s3  RB4
     * izquierda RC2
     * derecha   RC1 */
    
    TRISA = 0x00;               
    TRISB = 0x01;                
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
    
    // Configurar PWM
    PR2 = 250;              // Valor inicial de PR2
    CCP1CONbits.P1M = 0;    // PWM bits de salida
    CCP1CONbits.CCP1M = 0b00001100; // Se habilita PWM   
    CCP2CONbits.CCP2M = 0b00001100;   
    
    CCPR1L = 0x0F; 
    CCPR2L = 0x0F;
    CCP1CONbits.DC1B = 0;   // Bits menos significativos del Duty Cycle
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
    
    PIR1bits.TMR2IF = 0;    // Se limpia la bandera
    T2CONbits.T2CKPS1 = 1;  // Prescaler de 16
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1;   // Se enciende el TMR2
    
    while (!PIR1bits.TMR2IF); // Se espera una interrupción
    PIR1bits.TMR2IF = 0;

    //Configurar la interrupcion
    INTCONbits.GIE = 1;         // Se habilitan las interrupciones globales
    INTCONbits.PEIE = 1;        // Se habilitan las interrupciones perifericas
    I2C_Slave_Init(0x60);       // Direccion del esclavo 
}

void getRojo(void){
    PORTBbits.RB3 = 0;
    PORTBbits.RB4 = 0;
    __delay_ms(100);
    pulseIn();
}

void getVerde(void){
    PORTBbits.RB3 = 0;
    PORTBbits.RB4 = 1;
    __delay_ms(100);
    pulseIn();
}

void getAzul(void){
    PORTBbits.RB3 = 1;
    PORTBbits.RB4 = 1;
    __delay_ms(100);
    pulseIn();
}

void pulseIn(void){
    TMR1 = 0;                   // Limpiar registro
    while (RB0);                // Mientras el pulso este en alto espera
    T1CONbits.TMR1ON = 1;       // Cuando el pulso baja inicia TMR1
    while(!RB0);                // Mientras este bajo cuenta
    T1CONbits.TMR1ON = 0;       // Cuando se encienda el pulso se apaga el TMR1
}
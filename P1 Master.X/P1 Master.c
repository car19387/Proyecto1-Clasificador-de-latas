//******************************************************************************
//  Encabezado
//******************************************************************************
/* File: P1 Master.c
 * Device: PIC16F887
 * Author: Angel Cuellar & Jefry Carrasco
 * Descripción: Master en comunicación I2C, LCD para desplegar los valores
 * de los sensores en los slaves, comunicación serial con Adafruit.io
 * Creado: 17 de agosto, 2021
 * Modificado:  de agosto, 2021 */

//******************************************************************************
// Librerías incluidas
//******************************************************************************
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "I2C.h"
#include "LCD.h"

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
#define ENTER 13                // Enter en Ascii
#define PUNTO 46                // Punto en Ascii

//******************************************************************************
// Variables
//******************************************************************************
uint16_t Luz;                   // Valor de fotoresistencia
uint8_t CC ;                     // Cantidad de Coca Cola
uint8_t Sp ;                     // Cantidad de Sprite
uint8_t Tnk = 5;                // Gaseosas en el tanque
uint8_t Unidad;                 // Para conversion a decimal
uint8_t Decena;
uint8_t Centena;
unsigned char opcion;
unsigned char LZ[2];
char i; 
uint8_t distancia; 

//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void setup(void);               //Configuración
void Decimal(uint16_t var);     //Conversion a decimal

//******************************************************************************
// Main
//******************************************************************************
void main(void) {
    setup();                    // Iniciar configuraciones
    Lcd_Init();                 // Inicializar LCD
    Lcd_Clear();                // Limpiar LCD
    Lcd_Set_Cursor(1,1);        // Cursor en fila uno primera posición 
    Lcd_Write_String("CC: Sp: Lz: Tnk:");
    
    //**************************************************************************
    // Loop principal
    //**************************************************************************
    while(1){
        // Lectura de Coca Cola (Slave 1)
        I2C_Master_Start();         // Iniciar comunicación I2C
        I2C_Master_Write(0x60);     // Direccion de lectura del primer esclavo 
        I2C_Master_Write(0);        // Enviar dato de bandera
        I2C_Master_Stop();          // Terminar comunicación
        __delay_ms(200);
       
        I2C_Master_Start();         // Iniciar comunicación I2C
        I2C_Master_Write(0x61);     // Direccion de lectura del primer esclavo 
        CC = I2C_Master_Read(0);    // Guardar lectura
        I2C_Master_Stop();          // Terminar comunicación
        __delay_ms(200); 
        
        CC = CC;                    // Conversión de Coca Cola
        Decimal(CC);                // Conversion a decimal
        Lcd_Set_Cursor(2,1);        // Se agrega el valor a la LCD
        Lcd_Write_Char(Decena);
        Lcd_Write_Char(Unidad);
        Lcd_Write_String("  ");
        
        // Lectura de Sprite (Slave 1)
        I2C_Master_Start();         // Iniciar comunicación I2C
        I2C_Master_Write(0x60);     // Direccion de lectura del segundo esclavo 
        I2C_Master_Write(1);        // Enviar dato de bandera
        I2C_Master_Stop();          // Terminar comunicación
        __delay_ms(200);
        
        I2C_Master_Start();         // Iniciar comunicación I2C
        I2C_Master_Write(0x61);     // Direccion de lectura del segundo esclavo 
        Sp = I2C_Master_Read(0);    // Guardar lectura
        I2C_Master_Stop();          // Terminar comunicación
        __delay_ms(200); 
        
        Sp = Sp;                    // Conversión de Coca Cola
        Decimal(Sp);                // Conversion a decimal
        Lcd_Write_Char(Decena);
        Lcd_Write_Char(Unidad);
        Lcd_Write_String("  ");
        
        // Lectura de Luz (Slave 2)
        I2C_Master_Start();         // Iniciar comunicación I2C
        I2C_Master_Write(0x71);     // Direccion de lectura del primer esclavo  
        Luz = I2C_Master_Read(0);   // Guardar lectura                          
        I2C_Master_Stop();          // Terminar comunicación 
        __delay_ms(200);
        
        Luz = Luz/2.57;             // Conversion del valor del luz a 99%
        Decimal(Luz);               // Conversion a decimal
        LZ[0] = Decena;
        LZ[1] = Unidad; 
        Lcd_Write_Char(Decena);
        Lcd_Write_Char(Unidad);
        Lcd_Write_String("%");
        Lcd_Write_String(" ");
        
        // Lectura de nivel de Tanque (Slave 3)
        I2C_Master_Start();         // Iniciar comunicación I2C
        I2C_Master_Write(0x81);     // Direccion de lectura del segundo esclavo ////////////////////////////////////////////////////////////////
        Tnk = I2C_Master_Read(0);   // Guardar lectura
        I2C_Master_Stop();          // Terminar comunicación
        __delay_ms(200);
        
        Tnk = Tnk;                  // Conversión de Tnk
        Decimal(Tnk);               // Conversion a decimal
        Lcd_Write_Char(Centena);
        Lcd_Write_Char(Decena);
        Lcd_Write_Char(Unidad);
        
    }
    return;
}

//******************************************************************************
// Interrupciones
//******************************************************************************
void __interrupt()isr(void){
    //interrucion por recepcion de informacion USART 
    if (RCIF == 1) {
        opcion = RCREG;        // lo que reciba de la PC lo pongo en la variable
        switch (opcion) {
            case('a'):              // enviando cantidad de cocacola 
                TXREG = (CC+ 48); 
            break;
            case('b'):
                TXREG = (CC + 48); // enviando cantidad de sprite 
            break;
            case('c'): 
                distancia = (Tnk/7);         // enviando la cantidad de espacios 
                TXREG = (distancia + 48);    // disponibles   
            break;
            case('d'): 
                for (i = 0; i < strlen(LZ); i++) {
                __delay_ms(100);
                    if (TXIF == 1) {          // envio el porcentaje de intensidad 
                        TXREG = LZ[i];        // de luz del ambiente 
                    }
                }
            break;    
        }
        RCIF = 0;             // bajo la bandera
    }
    return;
}

//******************************************************************************
// Configuraciones
//******************************************************************************
void setup(void){
    // Configuración de Puertos
    ANSELH = 0x00;              //Pines digitales
    ANSEL = 0x00; 
    
    TRISA = 0x00;               // Puertos como salida
    TRISB = 0x00;
    TRISC = 0b10000000;
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
    I2C_Master_Init(100000);    // Comunicación I2C a 100KHz
    
    // configuracion para comunicacion USART 
    TXSTAbits.TX9 = 0; 
    TXSTAbits.TXEN = 1;   // configuracion de registro TXSTA 
    TXSTAbits.SYNC = 0; 
    TXSTAbits.BRGH = 1; 

    RCSTAbits.SPEN = 1; 
    RCSTAbits.RX9 = 0;    // configuracion de registro RCSTA 
    RCSTAbits.CREN = 1; 

    BAUDCTLbits.BRG16 = 1;   // configuracion de registro BAUDCTL

    SPBRG = 207; 
    SPBRGH = 0;           // configurando que opere a 9600 BAULIOS 
    
    // Configuracion de interrupciones
    INTCONbits.GIE = 1;         // Se habilitan las interrupciones globales
    INTCONbits.PEIE = 1;        // Se habilitan las interrupciones perifericas 
    PIE1bits.RCIE = 1; 
    PIR1bits.RCIF = 0;   // USART
}

//******************************************************************************
// Funciones
//******************************************************************************
void Decimal(uint16_t variable){        // Función para obtener valor decimal
    uint16_t valor;                     // Se define variable temporal
    valor = variable;                   // Se le asigna el valor recibido a var
    Centena = (valor/100) ;             // Valor del tercer digito
    valor = (valor - (Centena*100));     
    Decena = (valor/10);                // Valor del segundo digito
    valor = (valor - (Decena*10));
    Unidad = (valor);                   // Valor del primer digito
    
    Unidad = Unidad + 48;               // Conversion a ASCII
    Decena = Decena + 48;
    Centena = Centena + 48;
}
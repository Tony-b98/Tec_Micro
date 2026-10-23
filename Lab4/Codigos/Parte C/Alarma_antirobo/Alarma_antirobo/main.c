#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

#include "UART.h"

// CONFIGURACION 
#define USE_LCD            1        /* 0 si no hay LCD conectada (si no esta, el TWI se cuelga) */
#define PCF8574            0x27     

#define BAUD               9600UL
#define MY_UBRR            (F_CPU/16/BAUD-1)   

#define LDR_THRESHOLD      600      // umbral ADC (0-1023) - ajustar segun el divisor 
#define LDR_OPEN_WHEN_BRIGHT 1      // 1: ventana abierta = entra luz (ADC sube) 
#define PIR_ACTIVE_HIGH    1        // 1: PIR HC-SR501 (activo en alto); 0: boton a GND 

#define SENSOR_PERIOD_MS   20
#define SENSOR_CONFIRM     3        /* lecturas consecutivas para validar evento (antirrebote) */
#define STATUS_PERIOD_MS   5000UL   /* reporte periodico de estado por Bluetooth */
#define LCD_MIN_PERIOD_MS  100UL


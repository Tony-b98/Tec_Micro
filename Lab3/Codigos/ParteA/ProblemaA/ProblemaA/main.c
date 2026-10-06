#define F_CPU 16000000UL

#define PCF8574 0x27
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "twi_lcd.h"

// CONSTANTES Y VARIABLES GLOBALES

// Punto medio de confort 
#define PM_DEFECTO       20
#define PM_MIN           10
#define T_MAX_SENSOR     50    // segun datasheet DHT11 

// Margen necesario para que el rango superior (PM + OFFSET_ALTO)
// nunca supere la temperatura maxima admitida por el sensor.

#define MARGEN_SENSOR    25
#define PM_MAX           (T_MAX_SENSOR - MARGEN_SENSOR)

// Umbrales en funcion del PM 
#define OFFSET_CALEF      5
#define OFFSET_BAJO       5
#define OFFSET_MEDIO     15
#define OFFSET_ALTO      25

// Niveles de PWM reportados del ventilador 
#define PWM_BAJA        102
#define PWM_MEDIA       178
#define PWM_ALTA        255

// Sensor DHT11: pin PC0 = A0 
#define DHT_PORT    PORTC
#define DHT_DDR     DDRC
#define DHT_PINR    PINC
#define DHT_BIT     PC0

// LEDs indicadores en PORTB 
#define LED_CALEF   PB1        // D9: calefactor
#define LED_BAJA    PB2        // D10: ventilador baja vel.
#define LED_MEDIA   PB3        // D11: ventilador media vel.
#define LED_ALTA    PB4        // D12: ventilador alta vel. 

#define F_CPU 16000000UL
#define PCF8574 0x27  
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "twi_lcd.h"

#define N_COLORES 4
#define N_LEDS 1               // Cantidad REAL de LEDs de la tira 
#define WS2812_HABILITADA 1    // Solo afecta al modo WS2812 
#define WS_DDR_REG DDRD
#define WS_PORT_REG PORTD
#define WS_BIT (1 << PD6)
#define ADC_PROM 16
#define BARRIDOS_CAL 16
#define ESTABILIZACION_MS 180  // Aumentar si la LDR aun no se estabiliza 
#define MARGEN_ADC 8           // Margen inicial ajustable; NO valor medido 
#define PAGINA_MS 1800
#define SERVO_RECORRIDO_GRADOS 180
#define SERVO_PULSO_MIN_US 1000UL
#define SERVO_PULSO_MAX_US 2000UL
#define EEPROM_MAGIC 0xC404
#define EEPROM_VERSION 1

// 1 = LED RGB de 4 patas (prueba en mesa). 0 = WS2812 (laboratorio). 
#define MODO_RGB 1
#if MODO_RGB
#define BIT_R (1 << PD3)   // Rojo  -> D3 
#define BIT_G (1 << PD5)   // Verde -> D5
#define BIT_B (1 << PD6)   // Azul  -> D6 
#define RGB_MASK (BIT_R | BIT_G | BIT_B)
#endif

#if SERVO_RECORRIDO_GRADOS != 180 && SERVO_RECORRIDO_GRADOS != 360
#error Usar 180 o 360 grados de recorrido POSICIONAL
#endif

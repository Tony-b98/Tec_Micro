#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

#include "UART.h"
#include "UART.c"     
#include "twi.h"

#define PCF8574 0x27
#include "twi_lcd.h"

#define LED_PIN PB5
#define BUZZER_PIN PB0
#define BOTON_PIN PD2
#define DHT_PIN PD4

#define ACTIVO 0
#define STANDBY 1
#define APAGADO 2
#define EMERGENCIA 3

#define PERIODO_LED 200UL
#define PERIODO_LCD 500UL
#define PERIODO_DHT 2000UL
#define TIEMPO_BUZZER 200UL
#define TIEMPO_STANDBY 10000UL
#define TIEMPO_APAGADO 30000UL
#define TIEMPO_LARGO 3000UL
#define ANTIRREBOTE 50UL

#define UART_BUFFER 128U
#define LCD_CARACTERES 32U
#define LCD_CARACTER_MS 5UL

// VARIABLES GLOBALES

volatile uint32_t tiempo_sistema = 0;

volatile uint8_t solicitudes_adc = 0;
volatile uint16_t adc_solicitudes_perdidas = 0;
volatile uint8_t adc_listo = 0;
volatile uint16_t adc_resultado = 0;
volatile uint8_t adc_ocupado = 0;

volatile uint8_t flag_boton = 0;

volatile uint8_t uart_buffer[UART_BUFFER];
volatile uint8_t uart_cabeza = 0;
volatile uint8_t uart_cola = 0;
volatile uint16_t uart_tramas_perdidas = 0;

uint8_t estado = ACTIVO;
uint16_t valor_adc = 0;

uint8_t temperatura = 0;
uint8_t humedad = 0;
uint8_t temperatura_valida = 0;

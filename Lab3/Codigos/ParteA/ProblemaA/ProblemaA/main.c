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
// Variables de tiempo
volatile uint8_t segundos   = 0;
volatile uint8_t flag_medir = 0;

// Recepcion de comandos UART

volatile char     cmd_buf[16];
volatile uint8_t  cmd_idx     = 0;
volatile uint8_t  cmd_ready   = 0;
volatile uint8_t  cmd_invalid = 0;   // 1 = entrada rechazada
volatile uint8_t  cmd_p_cnt   = 0;   // conteo de 'P' de la linea

// Estado del sistema

uint8_t punto_medio = PM_DEFECTO;
uint8_t pwm_fan     = 0;
uint8_t calefactor  = 0;

// 1 mientras el usuario esta ingresando un nuevo PM
volatile uint8_t esperando_pm = 0;

// PROTOTIPOS DE FUNCIONES

// UART
static void uart_init(void);
static void uart_tx(char c);
static void uart_print(const char *s);

// DHT11
static uint8_t dht_esperar_mientras(uint8_t nivel);
static uint8_t dht11_leer(uint8_t *temp);

// LEDs
static void leds_init(void);
static void leds_off(void);
static void calef_set(uint8_t on);
static void fan_leds(uint8_t nivel);

// Timer1: base de 1 s
static void timer1_init(void);

// LCD
static void lcd_clear_safe(void);

// Logica de control
static uint8_t decidir_accion(uint8_t temperatura, uint8_t pm);
static const char *accion_txt(uint8_t accion);
static void aplicar_accion(uint8_t accion);

// LCD
static void lcd_mostrar(uint8_t temperatura);
static void lcd_error_sensor(uint8_t error);

// Menu UART
static void procesar_comando(void);

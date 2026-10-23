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

// PINES
// HC-05 - UART por software
#define BT_RX_PIN   PB2   // D10 recibe del HC-05
#define BT_TX_PIN   PB3   // D11 transmite al HC-05

// Sensor PIR
#define PIR_PIN     PD2   // D2

// Indicadores y buzzer
#define LED_G_PIN   PD3   // D3
#define LED_R_PIN   PD4   // D4
#define BUZ_PIN     PD5   // D5

// Pulsadores
#define BTN_ARM_PIN PD6   // D6
#define BTN_SIL_PIN PD7   // D7
#define BTN_MODE_PIN PB0  // D8

// Sensor LDR
#define LDR_CH      2     // A2 - ADC2

#if USE_LCD
#include "twi_lcd.h"
#endif

// TIPOS
typedef enum { ST_DESARMADO = 0, ST_ARMADO, ST_ALARMA, ST_SILENCIADA } state_t;
typedef enum { MODE_AUTO = 0, MODE_MANUAL } op_mode_t;
typedef enum { ZONE_NONE = 0, ZONE_WINDOW, ZONE_MOTION, ZONE_REMOTE } zone_t;
typedef enum { CMD_ARM, CMD_DISARM, CMD_SILENCE, CMD_AUTO, CMD_MANUAL, CMD_FORCE, CMD_STATUS } cmd_t;

static const char *const state_name[]  = { "DESARMADO", "ARMADO", "ALARMA", "SILENCIADA" };
static const char *const state_lcd[]   = { "DESARM.", "ARMADO", "ALARMA!", "SILENC." };
static const char *const zone_name[]   = { "NINGUNA", "VENTANA", "MOVIMIENTO", "REMOTO" };
static const char *const zone_lcd[]    = { "NINGUNA", "VENTANA", "MOVIM.", "REMOTO" };

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay_basic.h>
#include <stdint.h>
#include <stdio.h>
#include "twi.h"
#include "hc05.h"
#include "protocol.h"

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

#define WDT_MODO_DESPERTADOR 0U
#define WDT_MODO_RESET 1U
// Cambiar a WDT_MODO_RESET para compilar la prueba de recuperacion.
#ifndef WDT_MODO
#define WDT_MODO WDT_MODO_DESPERTADOR
#endif

#if WDT_MODO != WDT_MODO_DESPERTADOR && WDT_MODO != WDT_MODO_RESET
#error "WDT_MODO debe ser WDT_MODO_DESPERTADOR o WDT_MODO_RESET"
#endif

#define LCD_CARACTERES 32U
#define LCD_CARACTER_MS 5UL
#define LCD_TWI_TIMEOUT_MS 5UL
#define ADC_FILTER_SAMPLES 8U
#define ADC_RAW_SAMPLES 8U

#define LCD_TWI_LIBRE 0
#define LCD_TWI_ACTIVO 1
#define LCD_TWI_OK 2
#define LCD_TWI_ERROR 3
#define APAGADO_PRR_MASK (\
(1 << PRADC) | \
(1 << PRUSART0) | \
(1 << PRSPI) | \
(1 << PRTIM1) | \
(1 << PRTIM0) | \
(1 << PRTIM2) | \
(1 << PRTWI) \
)

// VARIABLES GLOBALES

volatile uint32_t tiempo_sistema = 0;
volatile uint8_t solicitudes_adc = 0;
volatile uint16_t adc_solicitudes_perdidas = 0;
static uint16_t adc_muestras[ADC_FILTER_SAMPLES];
static uint32_t adc_suma_muestras = 0;
static uint8_t adc_indice_muestra = 0;
static uint8_t adc_cantidad_muestras = 0;
volatile uint8_t flag_boton = 0;
volatile uint8_t wdt_despertar_pendiente = 0;
volatile uint16_t wdt_despertares = 0;

#if WDT_MODO == WDT_MODO_RESET
static uint8_t watchdog_reset_cause __attribute__((section(".noinit")));
static uint8_t watchdog_reset_test_active = 0;
#endif

volatile uint8_t estado = ACTIVO;
uint16_t valor_adc = 0;

uint8_t temperatura = 0;
uint8_t humedad = 0;
uint8_t temperatura_valida = 0;

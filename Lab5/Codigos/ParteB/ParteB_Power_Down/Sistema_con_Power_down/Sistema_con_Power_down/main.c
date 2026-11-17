#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay_basic.h>
#include <stdint.h>
#include <stdio.h>

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

volatile uint8_t estado = ACTIVO;
uint16_t valor_adc = 0;

uint8_t temperatura = 0;
uint8_t humedad = 0;
uint8_t temperatura_valida = 0;

// DHT11
#define DHT_INACTIVO 0
#define DHT_INICIO_LOW 1
#define DHT_RESP_LOW 2
#define DHT_RESP_HIGH 3
#define DHT_BIT_LOW 4
#define DHT_BIT_HIGH 5
#define DHT_TERMINADO 6
#define DHT_ERROR 7
#define DHT_MEDIR_HIGH 8

volatile uint8_t dht_estado = DHT_INACTIVO;
volatile uint8_t dht_datos[5];
volatile uint8_t dht_bits = 0;
volatile uint8_t dht_inicio_pulso = 0;

uint32_t dht_inicio_ms = 0;

// LCD - ACTUALIZACION DISTRIBUIDA

char lcd_objetivo[LCD_CARACTERES];
char lcd_enviado[LCD_CARACTERES];

uint8_t lcd_indice = 0;
uint8_t lcd_pendiente = 1;
volatile uint8_t lcd_twi_resultado = LCD_TWI_LIBRE;
volatile uint8_t lcd_twi_datos[10];
volatile uint8_t lcd_twi_indice = 0;
volatile uint8_t lcd_twi_caracter_indice = 0;
volatile uint8_t lcd_twi_caracter = 0;
uint32_t lcd_twi_inicio_ms = 0;
uint16_t lcd_twi_errores = 0;

#define ALARM_CAPACITY 8U

uint8_t alarm_codes[ALARM_CAPACITY];
uint8_t alarm_values[ALARM_CAPACITY];
uint8_t alarm_head = 0;
uint8_t alarm_count = 0;

// PROTOTIPOS
uint32_t millis_sistema(void);
void gpio_init(void);
void timers_init(void);
void adc_init(void);
void adc_poll(void);
void dht_iniciar(uint32_t ahora);
void dht_poll(uint32_t ahora);
static uint8_t power_down_wait_button(void);
static void sleep_idle(void);
static void timers_standby_init(void);
static uint16_t adc_read_average(void);
static uint16_t adc_convert_once(void);
static uint16_t adc_filtrar_muestra(uint16_t muestra);
void alarm_log(uint8_t code, uint8_t value);
static void lcd_copiar_linea(uint8_t fila, const char *texto);
void lcd_actualizar(void);
void lcd_poll(uint32_t ahora, uint32_t *t_caracter);
int main(void);

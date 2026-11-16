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
#define ALARM_WDT_WAKE 4U

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
void process_com_commands(
uint32_t ahora,
uint32_t *t_actividad,
uint32_t *t_led,
uint8_t *buzzer_temporizado,
uint8_t *apagado_aviso_enviado
);
uint8_t send_status(uint8_t response_command);
static void send_ack(uint8_t command);
static void send_nak(uint8_t command, const char *reason);
static void record_activity(
uint32_t ahora,
uint32_t *t_actividad,
uint32_t *t_led,
uint8_t *apagado_aviso_enviado,
uint8_t despertar_standby
);
static uint8_t power_down_wait_button(void);
static void sleep_idle(void);
static void timers_standby_init(void);
static void watchdog_interrupt_8s(void);
#if WDT_MODO == WDT_MODO_RESET
static void watchdog_reset_test(void);
static void watchdog_test_delay_ms(uint8_t ms);
static void watchdog_test_blink(uint8_t count);
#endif
static uint16_t adc_read_power_down(void);
static uint16_t adc_read_average(void);
static uint16_t adc_convert_once(void);
static uint16_t adc_filtrar_muestra(uint16_t muestra);
void alarm_log(uint8_t code, uint8_t value);
static void lcd_copiar_linea(uint8_t fila, const char *texto);
void lcd_actualizar(void);
void lcd_poll(uint32_t ahora, uint32_t *t_caracter);
int main(void);

#if WDT_MODO == WDT_MODO_RESET
void watchdog_early_init(void)
__attribute__((naked, section(".init3")));

void watchdog_early_init(void)
{
	watchdog_reset_cause = MCUSR;
	MCUSR = 0;
	wdt_disable();
}
#endif


// Implementacion de funciones

#if WDT_MODO == WDT_MODO_RESET
static void watchdog_reset_test(void)
{
	gpio_init();
	twi_init();
	twi_lcd_init();

	DDRB |= (1 << LED_PIN);
	PORTB &= ~(1 << LED_PIN);
	watchdog_test_blink(
	(watchdog_reset_cause & (1 << WDRF)) ? 2U : 1U
	);

	if (watchdog_reset_cause & (1 << WDRF))
	{
		twi_lcd_cmd(0x80);
		twi_lcd_msg("RESET WDT OK");
		twi_lcd_cmd(0xC0);
		twi_lcd_msg("Iniciando seguro");

		for (uint8_t i = 0; i < 10U; i++)
		{
			watchdog_test_delay_ms(200U);
		}

		return;
	}

	twi_lcd_cmd(0x80);
	twi_lcd_msg("PRUEBA WDT RESET");
	twi_lcd_cmd(0xC0);
	twi_lcd_msg("Sistema inicia");

	wdt_enable(WDTO_8S);
	watchdog_reset_test_active = 1;
}

static void watchdog_test_delay_ms(uint8_t ms)
{
	while (ms--)
	{
		_delay_loop_2(F_CPU / 4000UL);
	}
}

static void watchdog_test_blink(uint8_t count)
{
	for (uint8_t i = 0; i < count; i++)
	{
		PORTB |= (1 << LED_PIN);
		watchdog_test_delay_ms(150U);
		PORTB &= ~(1 << LED_PIN);
		watchdog_test_delay_ms(150U);
	}
}
#endif

static void sleep_idle(void)
{
	set_sleep_mode(SLEEP_MODE_IDLE);
	cli();
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
}

static void timers_standby_init(void)
{
	TCCR0A = (1 << WGM01);
	TCCR0B = 0;
	TCNT0 = 0;
	OCR0A = 155;
	TIFR0 = (1 << OCF0A);
	TIMSK0 = (1 << OCIE0A);
	TCCR0B = (1 << CS02) | (1 << CS00);
}

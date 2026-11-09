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


// BASE TEMPORAL
uint32_t millis_sistema(void)
{
	uint8_t s = SREG;
	cli();

	uint32_t t = tiempo_sistema;

	SREG = s;
	return t;
}


// UART PARA MATLAB

void uart_led(uint32_t marca_ms, uint8_t nivel)
{
	char trama[30];

	int n = snprintf(
	trama,
	sizeof(trama),
	"LED,%lu,%u\n",
	(unsigned long)marca_ms,
	(unsigned)nivel
	);

	if (n <= 0 || n >= (int)sizeof(trama))
	return;

	uint8_t s = SREG;
	cli();

	uint8_t libres = (uint8_t)(
	(uart_cola - uart_cabeza - 1U) &
	(UART_BUFFER - 1U)
	);

	if (libres < (uint8_t)n)
	{
		uart_tramas_perdidas++;
		SREG = s;
		return;
	}

	for (uint8_t i = 0; i < (uint8_t)n; i++)
	{
		uart_buffer[uart_cabeza] = (uint8_t)trama[i];

		uart_cabeza = (uint8_t)(
		(uart_cabeza + 1U) &
		(UART_BUFFER - 1U)
		);
	}

	UCSR0B |= (1 << UDRIE0);
	SREG = s;
}

ISR(USART_UDRE_vect)
{
	if (uart_cola != uart_cabeza)
	{
		UDR0 = uart_buffer[uart_cola];

		uart_cola = (uint8_t)(
		(uart_cola + 1U) &
		(UART_BUFFER - 1U)
		);
	}
	else
	{
		UCSR0B &= ~(1 << UDRIE0);
	}
}

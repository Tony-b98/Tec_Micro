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

// INTERRUPCIONES DE TEMPORIZADORES

ISR(TIMER0_COMPA_vect)
{
	tiempo_sistema++;
}

ISR(TIMER1_COMPA_vect)
{
	if (solicitudes_adc < 255)
	solicitudes_adc++;
	else if (adc_solicitudes_perdidas < 65535)
	adc_solicitudes_perdidas++;
}

ISR(ADC_vect)
{
	adc_resultado = ADC;
	adc_listo = 1;
	adc_ocupado = 0;
}

// INTERRUPCION EXTERNA INT0


ISR(INT0_vect)
{
	flag_boton = 1;
}
// =====================================================
// INTERRUPCION DHT11
// =====================================================

ISR(PCINT2_vect)
{
	if (dht_estado < DHT_RESP_LOW ||
	dht_estado > DHT_MEDIR_HIGH)
	return;

	uint8_t alto =
	(PIND & (1 << DHT_PIN)) != 0;

	uint8_t tic = TCNT2;

	if (dht_estado == DHT_RESP_LOW)
	{
		if (!alto)
		dht_estado = DHT_RESP_HIGH;
	}
	else if (dht_estado == DHT_RESP_HIGH)
	{
		if (alto)
		dht_estado = DHT_BIT_LOW;
	}
	else if (dht_estado == DHT_BIT_LOW)
	{
		if (!alto)
		dht_estado = DHT_BIT_HIGH;
	}
	else if (dht_estado == DHT_BIT_HIGH)
	{
		if (alto)
		{
			dht_inicio_pulso = tic;
			dht_estado = DHT_MEDIR_HIGH;
		}
	}
	else if (dht_estado == DHT_MEDIR_HIGH && !alto)
	{
		uint8_t duracion = (uint8_t)(
		tic - dht_inicio_pulso
		);

		if (duracion < 25U || duracion > 200U)
		{
			dht_estado = DHT_ERROR;
			return;
		}

		uint8_t pos = dht_bits / 8U;

		dht_datos[pos] = (uint8_t)(
		(dht_datos[pos] << 1) |
		(duracion >= 100U)
		);

		dht_bits++;

		dht_estado =
		(dht_bits == 40U) ?
		DHT_TERMINADO :
		DHT_BIT_HIGH;
	}
}

// =====================================================
// CONFIGURACION GPIO
// =====================================================

void gpio_init(void)
{
	DDRB |= (1 << LED_PIN) | (1 << BUZZER_PIN);

	PORTB &= ~(
	(1 << LED_PIN) |
	(1 << BUZZER_PIN)
	);

	DDRD &= ~(
	(1 << BOTON_PIN) |
	(1 << DHT_PIN)
	);

	PORTD |=
	(1 << BOTON_PIN) |
	(1 << DHT_PIN);

	DDRC &= ~(1 << PC0);
	PORTC &= ~(1 << PC0);

	// INT0: flanco descendente
	EICRA = (1 << ISC01);
	EIFR = (1 << INTF0);
	EIMSK = (1 << INT0);

	// DHT11: PCINT20
	PCICR &= ~(1 << PCIE2);
	PCMSK2 |= (1 << PCINT20);
}

// =====================================================
// CONFIGURACION TEMPORIZADORES
// =====================================================

void timers_init(void)
{
	// TIMER0: base de 1 ms
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);

	TCNT0 = 0;
	OCR0A = 249;
	TIMSK0 = (1 << OCIE0A);

	// TIMER1: interrupcion cada 100 ms
	TCCR1A = 0;

	TCCR1B =
	(1 << WGM12) |
	(1 << CS11) |
	(1 << CS10);

	TCNT1 = 0;
	OCR1A = 24999;
	TIMSK1 = (1 << OCIE1A);

	// TIMER2: resolucion de 0.5 us
	TCCR2A = 0;
	TCCR2B = (1 << CS21);
	TCNT2 = 0;
}

// =====================================================
// ADC
// =====================================================

void adc_init(void)
{
	ADMUX = (1 << REFS0);
	ADCSRB = 0;

	DIDR0 |= (1 << ADC0D);

	ADCSRA =
	(1 << ADEN) |
	(1 << ADIE) |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);
}

void adc_poll(void)
{
	if (adc_listo)
	{
		uint8_t s = SREG;
		cli();

		valor_adc = adc_resultado;
		adc_listo = 0;

		SREG = s;
	}

	if (!adc_ocupado && solicitudes_adc)
	{
		uint8_t s = SREG;
		cli();

		if (!adc_ocupado && solicitudes_adc)
		{
			solicitudes_adc--;
			adc_ocupado = 1;

			ADCSRA |= (1 << ADSC);
		}

		SREG = s;
	}
}

// =====================================================
// DHT11 - INICIO DE MEDICION
// =====================================================

void dht_iniciar(uint32_t ahora)
{
	for (uint8_t i = 0; i < 5; i++)
	dht_datos[i] = 0;

	dht_bits = 0;
	dht_inicio_ms = ahora;
	dht_estado = DHT_INICIO_LOW;

	PCICR &= ~(1 << PCIE2);

	PORTD &= ~(1 << DHT_PIN);
	DDRD |= (1 << DHT_PIN);
}

// =====================================================
// DHT11 - PROCESAMIENTO
// =====================================================

void dht_poll(uint32_t ahora)
{
	if (
	dht_estado == DHT_INICIO_LOW &&
	(uint32_t)(ahora - dht_inicio_ms) >= 20UL
	)
	{
		uint8_t s = SREG;
		cli();

		PORTD |= (1 << DHT_PIN);
		DDRD &= ~(1 << DHT_PIN);

		dht_estado = DHT_RESP_LOW;

		PCIFR = (1 << PCIF2);
		PCICR |= (1 << PCIE2);

		SREG = s;
	}

	if (
	dht_estado == DHT_TERMINADO ||
	dht_estado == DHT_ERROR ||
	(
	dht_estado != DHT_INACTIVO &&
	(uint32_t)(ahora - dht_inicio_ms) > 35UL
	)
	)
	{
		uint8_t s = SREG;
		cli();

		PCICR &= ~(1 << PCIE2);

		uint8_t listo =
		(dht_estado == DHT_TERMINADO);

		uint8_t bytes[5];

		for (uint8_t i = 0; i < 5; i++)
		bytes[i] = dht_datos[i];

		dht_estado = DHT_INACTIVO;

		SREG = s;

		uint8_t suma = (uint8_t)(
		bytes[0] +
		bytes[1] +
		bytes[2] +
		bytes[3]
		);

		if (listo && suma == bytes[4])
		{
			humedad = bytes[0];
			temperatura = bytes[2];
			temperatura_valida = 1;
		}
		else
		{
			temperatura_valida = 0;
		}
	}
}

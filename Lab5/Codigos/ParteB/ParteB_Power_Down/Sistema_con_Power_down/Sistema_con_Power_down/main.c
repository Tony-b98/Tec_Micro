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

static uint8_t power_down_wait_button(void)
{
	if (
	lcd_pendiente ||
	lcd_twi_resultado == LCD_TWI_ACTIVO
	)
	{
		return 0;
	}

	lcd &= ~(1 << 3);
	PCF8574_write(lcd);
	while (TWCR & (1 << TWSTO))
	{
	}

	cli();
	flag_boton = 0;
	PCICR &= ~(1 << PCIE2);
	TIMSK0 = 0;
	TIMSK1 = 0;
	TIMSK2 = 0;
	TCCR0A = 0;
	TCCR0B = 0;
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR2A = 0;
	TCCR2B = 0;
	ADCSRA = 0;
	TWCR = 0;
	UCSR0B = 0;
	ACSR |= (1 << ACD);
	PRR |= APAGADO_PRR_MASK;

	EICRA = 0;
	EIFR = (1 << INTF0);
	EIMSK = (1 << INT0);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	if (!(PIND & (1 << BOTON_PIN)))
	{
		flag_boton = 1;
	}

	// Power-Down sin despertar periodico: la unica fuente
	// de despertar es INT0 (nivel bajo del pulsador).
	while (!flag_boton)
	{
		sleep_enable();
		sleep_bod_disable();
		sei();
		sleep_cpu();
		sleep_disable();
		cli();
	}

	EIMSK = 0;
	PRR &= ~APAGADO_PRR_MASK;
	ACSR &= ~(1 << ACD);

	timers_init();
	adc_init();
	twi_init();
	DDRC &= ~(1 << PC0);
	PORTC &= ~(1 << PC0);
	twi_lcd_cmd(0x0C);

	dht_estado = DHT_INACTIVO;
	EICRA = (1 << ISC01);
	EIFR = (1 << INTF0);
	EIMSK = (1 << INT0);

	estado = ACTIVO;
	sei();
	return 1;
}

void alarm_log(uint8_t code, uint8_t value)
{
	uint8_t index;

	if (alarm_count < ALARM_CAPACITY)
	{
		index = (uint8_t)(
		(alarm_head + alarm_count) % ALARM_CAPACITY
		);
		alarm_count++;
	}
	else
	{
		index = alarm_head;
		alarm_head = (uint8_t)(
		(alarm_head + 1U) % ALARM_CAPACITY
		);
	}

	alarm_codes[index] = code;
	alarm_values[index] = value;
}

// BASE TEMPORAL
uint32_t millis_sistema(void)
{
	uint8_t s = SREG;
	cli();

	uint32_t t = tiempo_sistema;

	SREG = s;
	return t;
}

// INTERRUPCIONES DE TEMPORIZADORES
ISR(TIMER0_COMPA_vect)
{
	if (estado == STANDBY)
	{
		tiempo_sistema += 10UL;
	}
	else
	{
		tiempo_sistema++;
	}
}

ISR(TIMER1_COMPA_vect)
{
	if (estado == ACTIVO)
	{
		if (solicitudes_adc < 255)
		{
			solicitudes_adc++;
		}
		else if (adc_solicitudes_perdidas < 65535)
		{
			adc_solicitudes_perdidas++;
		}
	}
}

// INTERRUPCION EXTERNA INT0
ISR(INT0_vect)
{
	flag_boton = 1;
}

// INTERRUPCION DHT11
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

// CONFIGURACION GPIO
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

// CONFIGURACION TEMPORIZADORES
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

// ADC
void adc_init(void)
{
	uint8_t s = SREG;
	cli();

	PRR &= ~(1 << PRADC);
	ADCSRA = 0;
	ADMUX = (1 << REFS0);
	ADCSRB = 0;
	DIDR0 |= (1 << ADC0D);

	solicitudes_adc = 0;
	adc_suma_muestras = 0;
	adc_indice_muestra = 0;
	adc_cantidad_muestras = 0;

	ADCSRA =
	(1 << ADEN) |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);

	(void)adc_convert_once();

	SREG = s;
}

void adc_poll(void)
{
	uint8_t s = SREG;
	cli();

	uint8_t leer = (solicitudes_adc != 0);
	solicitudes_adc = 0;

	SREG = s;

	if (!leer)
	{
		return;
	}

	ADMUX = (1 << REFS0);
	valor_adc = adc_filtrar_muestra(adc_read_average());
}

static uint16_t adc_convert_once(void)
{
	ADCSRA |= (1 << ADSC);

	while (ADCSRA & (1 << ADSC))
	{
	}

	return ADC;
}

static uint16_t adc_read_average(void)
{
	uint32_t suma = 0;

	for (uint8_t i = 0; i < ADC_RAW_SAMPLES; i++)
	{
		suma += adc_convert_once();
	}

	return (uint16_t)(
	(suma + (ADC_RAW_SAMPLES / 2U)) / ADC_RAW_SAMPLES
	);
}

static uint16_t adc_filtrar_muestra(uint16_t muestra)
{
	if (adc_cantidad_muestras < ADC_FILTER_SAMPLES)
	{
		adc_muestras[adc_indice_muestra] = muestra;
		adc_suma_muestras += muestra;
		adc_cantidad_muestras++;
	}
	else
	{
		adc_suma_muestras -= adc_muestras[adc_indice_muestra];
		adc_muestras[adc_indice_muestra] = muestra;
		adc_suma_muestras += muestra;
	}

	adc_indice_muestra++;
	if (adc_indice_muestra >= ADC_FILTER_SAMPLES)
	{
		adc_indice_muestra = 0;
	}

	return (uint16_t)(
	adc_suma_muestras / adc_cantidad_muestras
	);
}
// FUNCION PRINCIPAL
int main(void)
{
	cli();
	// WDT siempre deshabilitado: no se usa ni como despertador ni como reset.
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	gpio_init();
	timers_init();
	adc_init();

	twi_init();

	// Mantener A0 como entrada analogica
	DDRC &= ~(1 << PC0);
	PORTC &= ~(1 << PC0);

	sei();

	twi_lcd_init();
	twi_lcd_cmd(0x0C);

	for (uint8_t i = 0; i < LCD_CARACTERES; i++)
	{
		lcd_enviado[i] = '\0';
	}

	lcd_actualizar();

	uint32_t ahora = millis_sistema();

	uint32_t t_led = ahora;
	uint32_t t_lcd = ahora;
	uint32_t t_lcd_caracter = ahora;
	uint32_t t_dht = ahora;
	uint32_t t_actividad = ahora;
	uint32_t t_buzzer = ahora;
	uint32_t t_raw = ahora;
	uint32_t t_presion = ahora;

	uint8_t raw_prev =
	(PIND & (1 << BOTON_PIN)) ? 1 : 0;

	uint8_t boton_estable = raw_prev;

	uint8_t pulsacion_larga = 0;
	uint8_t recuperar_emergencia = 0;
	uint8_t buzzer_temporizado = 0;
	uint8_t pulsacion_por_int0 = 0;
	uint8_t estado_anterior = estado;
	uint8_t standby_lcd_configurado = 0;
	while (1)
	{
		ahora = millis_sistema();
		// PULSADOR - ANTIRREBOTE E INT0
		uint8_t raw =
		(PIND & (1 << BOTON_PIN)) ? 1 : 0;

		if (raw != raw_prev)
		{
			raw_prev = raw;
			t_raw = ahora;
		}

		if (
		raw != boton_estable &&
		(uint32_t)(ahora - t_raw) >= ANTIRREBOTE
		)
		{
			boton_estable = raw;

			if (!boton_estable)
			{
				uint8_t s = SREG;
				cli();

				pulsacion_por_int0 = flag_boton;
				flag_boton = 0;

				SREG = s;

				if (pulsacion_por_int0)
				{
					t_presion = ahora;
					pulsacion_larga = 0;
					recuperar_emergencia = 0;
				}
			}
			else if (pulsacion_por_int0)
			{
				if (!pulsacion_larga)
				{
					if (estado != EMERGENCIA)
					{
						estado = ACTIVO;
						t_actividad = ahora;

						PORTB |= (1 << BUZZER_PIN);

						buzzer_temporizado = 1;
						t_buzzer = ahora;
					}
				}
				else if (
				estado == EMERGENCIA &&
				recuperar_emergencia
				)
				{
					estado = ACTIVO;
					t_actividad = ahora;
					t_led = ahora;

					recuperar_emergencia = 0;
				}

				pulsacion_por_int0 = 0;
			}
		}

		// PULSACION LARGA - EMERGENCIA
		if (
		!boton_estable &&
		pulsacion_por_int0 &&
		!pulsacion_larga &&
		(uint32_t)(ahora - t_presion) >= TIEMPO_LARGO
		)
		{
			pulsacion_larga = 1;

			if (estado != EMERGENCIA)
			{
				estado = EMERGENCIA;

				PORTB &= ~(
				(1 << LED_PIN) |
				(1 << BUZZER_PIN)
				);

				buzzer_temporizado = 0;
				recuperar_emergencia = 0;
			}
			else
			{
				recuperar_emergencia = 1;
			}
		}

		// BUZZER
		if (
		buzzer_temporizado &&
		(uint32_t)(ahora - t_buzzer) >= TIEMPO_BUZZER
		)
		{
			PORTB &= ~(1 << BUZZER_PIN);
			buzzer_temporizado = 0;
		}

		// MAQUINA DE ESTADOS
		if (
		estado == ACTIVO &&
		(uint32_t)(ahora - t_actividad) >=
		TIEMPO_STANDBY
		)
		{
			estado = STANDBY;
		}
		else if (estado == STANDBY)
		{
			uint32_t inactividad =
			(uint32_t)(ahora - t_actividad);

			if (inactividad >= TIEMPO_APAGADO)
			{
				estado = APAGADO;
			}
		}

		if (estado != estado_anterior)
		{
			uint8_t estado_previo = estado_anterior;
			estado_anterior = estado;

			if (estado == STANDBY)
			{
				PORTB &= ~(1 << BUZZER_PIN);
				buzzer_temporizado = 0;
				PCICR &= ~(1 << PCIE2);
				dht_estado = DHT_INACTIVO;
				ADCSRA &= ~(1 << ADEN);
				TIMSK1 = 0;
				TCCR1B = 0;
				PRR |=
				(1 << PRADC) |
				(1 << PRTIM1) |
				(1 << PRTIM2);
				timers_standby_init();
				standby_lcd_configurado = 0;
			}
			else if (estado == APAGADO)
			{
				PORTB &= ~(1 << BUZZER_PIN);
				buzzer_temporizado = 0;
			}
			else if (
			estado_previo == STANDBY &&
			estado != APAGADO
			)
			{
				PRR &= ~(
				(1 << PRADC) |
				(1 << PRTIM1) |
				(1 << PRTIM2)
				);
				timers_init();
				adc_init();
				standby_lcd_configurado = 0;
			}

			alarm_log(1U, estado);

			for (uint8_t i = 0; i < LCD_CARACTERES; i++)
			{
				lcd_enviado[i] = '\0';
			}

			lcd_actualizar();
			t_lcd = ahora;
		}

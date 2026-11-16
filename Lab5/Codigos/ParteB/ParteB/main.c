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
static void watchdog_interrupt_8s(void)
{
	MCUSR &= ~(1 << WDRF);
	wdt_reset();
	WDTCSR = (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
}

static uint16_t adc_read_power_down(void)
{
	PRR &= ~(1 << PRADC);
	DDRC &= ~(1 << PC0);
	PORTC &= ~(1 << PC0);
	ADMUX = (1 << REFS0);
	ADCSRB = 0;
	DIDR0 |= (1 << ADC0D);
	ADCSRA =
	(1 << ADEN) |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);

	(void)adc_convert_once();
	uint16_t value = adc_read_average();
	ADCSRA = 0;
	PRR |= (1 << PRADC);
	return value;
}

static uint8_t power_down_wait_button(void)
{
	if (
	!hc05_tx_done() ||
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
	DDRB |= (1 << PB1);
	PORTB &= ~(1 << PB1);
	watchdog_interrupt_8s();

	if (!(PIND & (1 << BOTON_PIN)))
	{
		flag_boton = 1;
	}

	while (!flag_boton)
	{
		wdt_despertar_pendiente = 0;
		sleep_enable();
		sleep_bod_disable();
		sei();
		sleep_cpu();
		sleep_disable();
		cli();

		if (wdt_despertar_pendiente)
		{
			wdt_despertar_pendiente = 0;
			sei();
			valor_adc = adc_filtrar_muestra(adc_read_power_down());
			wdt_despertares++;
			alarm_log(ALARM_WDT_WAKE, (uint8_t)(valor_adc >> 2));
			cli();
		}
	}

	sleep_disable();
	wdt_disable();
	EIMSK = 0;
	PRR &= ~APAGADO_PRR_MASK;
	ACSR &= ~(1 << ACD);

	timers_init();
	adc_init();
	twi_init();
	DDRC &= ~(1 << PC0);
	PORTC &= ~(1 << PC0);
	twi_lcd_cmd(0x0C);
	hc05_init(103);
	protocol_init();

	dht_estado = DHT_INACTIVO;
	EICRA = (1 << ISC01);
	EIFR = (1 << INTF0);
	EIMSK = (1 << INT0);

	estado = ACTIVO;
	sei();
	return 1;
}

// COMANDOS DE COMUNICACION CON TRAMAS ASCII BR
static void send_ack(uint8_t command)
{
	char data[2] = {(char)command, '\0'};
	(void)protocol_send_frame(COM_RESP_ACK, data);
}

static void send_nak(uint8_t command, const char *reason)
{
	char data[COM_MAX_DATA + 1U];
	(void)snprintf(
	data,
	sizeof(data),
	"%c,%s",
	command ? (char)command : '?',
	reason
	);

	(void)protocol_send_frame(COM_RESP_NAK, data);
}

static void record_activity(
uint32_t ahora,
uint32_t *t_actividad,
uint32_t *t_led,
uint8_t *apagado_aviso_enviado,
uint8_t despertar_standby
)
{
	*t_actividad = ahora;
	*apagado_aviso_enviado = 0;

	if (despertar_standby && estado == STANDBY)
	{
		estado = ACTIVO;
		*t_led = ahora;
	}
}

uint8_t send_status(uint8_t response_command)
{
	char data[COM_MAX_DATA + 1U];

	if (response_command == COM_RESP_STATUS)
	{
		const char *estado_texto = "APAGADO";

		if (estado == ACTIVO)
		{
			estado_texto = "ACTIVO";
		}
		else if (estado == STANDBY)
		{
			estado_texto = "STANDBY";
		}
		else if (estado == EMERGENCIA)
		{
			estado_texto = "EMERGENCIA";
		}

		(void)snprintf(
		data,
		sizeof(data),
		"%s,%u,%u,%u,%u",
		estado_texto,
		(unsigned)valor_adc,
		(unsigned)temperatura,
		(unsigned)humedad,
		(unsigned)temperatura_valida
		);
	}
	else
	{
		(void)snprintf(
		data,
		sizeof(data),
		"%u,%u,%u,%u",
		(unsigned)valor_adc,
		(unsigned)temperatura,
		(unsigned)humedad,
		(unsigned)temperatura_valida
		);
	}

	return protocol_send_frame(response_command, data);
}

void process_com_commands(
uint32_t ahora,
uint32_t *t_actividad,
uint32_t *t_led,
uint8_t *buzzer_temporizado,
uint8_t *apagado_aviso_enviado
)
{
	com_command_t command;

	while (protocol_get_command(&command))
	{
		if (command.command == COM_CMD_STATUS)
		{
			if (command.data[0] != '\0')
			{
				send_nak(command.command, "LENGTH");
			}
			else
			{
				record_activity(
				ahora,
				t_actividad,
				t_led,
				apagado_aviso_enviado,
				0U
				);

				if (!send_status(COM_RESP_STATUS))
				{
					send_nak(command.command, "BUSY");
				}
			}
		}
		else if (command.command == COM_CMD_READ)
		{
			if (command.data[0] != '\0')
			{
				send_nak(command.command, "LENGTH");
			}
			else
			{
				record_activity(
				ahora,
				t_actividad,
				t_led,
				apagado_aviso_enviado,
				0U
				);

				if (!send_status(COM_RESP_READ))
				{
					send_nak(command.command, "BUSY");
				}
			}
		}
		else if (command.command == COM_CMD_MODE)
		{
			if (command.data[0] < '0' ||
			command.data[0] > '2' ||
			command.data[1] != '\0')
			{
				send_nak(command.command, "DATA");
			}
			else if (estado == EMERGENCIA)
			{
				send_nak(command.command, "DENIED");
			}
			else
			{
				estado = (uint8_t)(command.data[0] - '0');
				*t_actividad = ahora;
				*apagado_aviso_enviado = 0;

				if (estado == ACTIVO)
				{
					*t_led = ahora;
				}

				send_ack(command.command);
			}
		}
		else if (command.command == COM_CMD_ACTUATOR)
		{
			if ((command.data[0] != '0' && command.data[0] != '1') ||
			command.data[1] != '\0')
			{
				send_nak(command.command, "DATA");
			}
			else if (estado == EMERGENCIA)
			{
				send_nak(command.command, "DENIED");
			}
			else
			{
				if (command.data[0] == '1')
				{
					PORTB |= (1 << BUZZER_PIN);
				}
				else
				{
					PORTB &= ~(1 << BUZZER_PIN);
				}

				*buzzer_temporizado = 0;
				record_activity(
				ahora,
				t_actividad,
				t_led,
				apagado_aviso_enviado,
				1U
				);
				send_ack(command.command);
			}
		}
		else if (command.command == COM_CMD_EMERGENCY)
		{
			if (command.data[0] != '\0')
			{
				send_nak(command.command, "LENGTH");
			}
			else
			{
				record_activity(
				ahora,
				t_actividad,
				t_led,
				apagado_aviso_enviado,
				0U
				);

				if (estado != EMERGENCIA)
				{
					estado = EMERGENCIA;
					PORTB &= ~(
					(1 << LED_PIN) |
					(1 << BUZZER_PIN)
					);
					*buzzer_temporizado = 0;
				}

				send_ack(command.command);
			}
		}
		else if (command.command == COM_CMD_ALARMS)
		{
			if (command.data[0] != '\0')
			{
				send_nak(command.command, "LENGTH");
				continue;
			}

			record_activity(
			ahora,
			t_actividad,
			t_led,
			apagado_aviso_enviado,
			0U
			);

			char data[COM_MAX_DATA + 1U];
			size_t used = 0U;
			uint8_t failed = 0U;

			for (uint8_t i = 0; i < alarm_count; i++)
			{
				uint8_t index = (uint8_t)(
				(alarm_head + i) % ALARM_CAPACITY
				);
				int written = snprintf(
				data + used,
				sizeof(data) - used,
				"%s%u,%u",
				i ? ";" : "",
				(unsigned)alarm_codes[index],
				(unsigned)alarm_values[index]
				);

				if (
				written < 0 ||
				(size_t)written >= sizeof(data) - used
				)
				{
					failed = 1U;
					break;
				}

				used += (size_t)written;
			}

			if (failed)
			{
				send_nak(command.command, "OVERFLOW");
				continue;
			}

			if (!used)
			{
				(void)snprintf(data, sizeof(data), "END");
			}

			if (!protocol_send_frame(COM_RESP_ALARM, data))
			{
				send_nak(command.command, "BUSY");
			}
		}
		else
		{
			send_nak(command.command, "UNKNOWN");
		}
	}
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

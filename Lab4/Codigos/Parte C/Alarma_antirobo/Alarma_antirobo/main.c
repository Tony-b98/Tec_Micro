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

//VARIABLES GLOBALES
static volatile uint32_t g_millis = 0;

#define RX_BUF_SIZE 64
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static uint8_t rx_tail = 0;

#define LINE_MAX_LEN 24
static char line[LINE_MAX_LEN];
static uint8_t line_len = 0;

static state_t   state = ST_DESARMADO;
static op_mode_t mode  = MODE_AUTO;
static zone_t    zone  = ZONE_NONE;

static uint8_t ldr_cnt = 0, pir_cnt = 0;
static uint8_t prev_arm = 0, prev_sil = 0, prev_mode = 0;

static char last_rx[4] = "---";
static uint8_t lcd_dirty = 1;

// PROTOTIPOS DE FUNCIONES 
static void timer0_init(void);
static uint32_t millis(void);
static void adc_init(void);
static uint16_t adc_read(uint8_t ch);
static void send_status(void);
static void set_state(state_t s);
static void on_event(zone_t z);
static void handle_command(cmd_t cmd);
static void process_line(const char *l);
static void bt_poll(void);
static uint8_t debounce(uint8_t active, uint8_t *cnt);
static uint8_t pir_active(void);
static uint8_t ldr_open(void);
static void sensors_poll(void);
static uint8_t edge(uint8_t now, uint8_t *prev);
static void buttons_poll(void);
static void update_outputs(uint32_t now);
#if USE_LCD
static void lcd_refresh(void);
#endif
static void io_init(void);


// IMPLEMENTACION DE FUNCIONES 

ISR(TIMER0_COMPA_vect) {
	g_millis++;
}

static void timer0_init(void) {
	TCCR0A = (1 << WGM01);                 // CTC 
	TCCR0B = (1 << CS01) | (1 << CS00);    // prescaler 64 -> 250 kHz
	OCR0A  = 249;                          // 250 ticks = 1 ms 
	TIMSK0 = (1 << OCIE0A);
}

static uint32_t millis(void) {
	uint32_t m;
	uint8_t s = SREG;
	cli();
	m = g_millis;
	SREG = s;
	return m;
}


// UART RX por interrupcion 
ISR(USART_RX_vect) {
	uint8_t d = UDR0;
	uint8_t next = (rx_head + 1) % RX_BUF_SIZE;
	if (next != rx_tail) {
		rx_buf[rx_head] = d;
		rx_head = next;
	}
}

//  ADC 
static void adc_init(void) {
	ADMUX  = (1 << REFS0);                    // AVcc como referencia
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 
}

static uint16_t adc_read(uint8_t ch) {
	ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}
// TRAMAS HACIA EL CELULAR 
static void send_status(void) {
	uart_print("ALARMA:");
	uart_print(state == ST_ALARMA ? "ON" : "OFF");
	uart_print(";ZONA:");
	uart_print(zone_name[zone]);
	uart_print(";ESTADO:");
	uart_print(state_name[state]);
	uart_print(";MODO:");
	uart_print(mode == MODE_AUTO ? "AUTO" : "MANUAL");
	uart_print("\r\n");
}

//MAQUINA DE ESTADOS 
static void set_state(state_t s) {
	state = s;
	if (s == ST_ARMADO) {
		ldr_cnt = 0;            //si ya hay un sensor activo, genera evento al armar 
		pir_cnt = 0;
		zone = ZONE_NONE;
		} else if (s == ST_DESARMADO) {
		zone = ZONE_NONE;
	}
	lcd_dirty = 1;
	send_status();
}

// Evento de intrusion detectado por un sensor (solo llega en estado ARMADO) 
static void on_event(zone_t z) {
	zone = z;
	lcd_dirty = 1;
	if (mode == MODE_AUTO) {
		set_state(ST_ALARMA);       //AUTO: la planta dispara sola y notifica 
		} else {
		uart_print("EVENTO:DETECTADO;ZONA:");   //MANUAL: solo avisa, decide el operador 
		uart_print(zone_name[z]);
		uart_print("\r\n");
	}
}

// Comandos: los usan tanto el Bluetooth como los botones 
static void handle_command(cmd_t cmd) {
	switch (cmd) {
		case CMD_ARM:
		if (state == ST_DESARMADO || state == ST_SILENCIADA) set_state(ST_ARMADO);
		else send_status();
		break;

		case CMD_DISARM:
		if (state != ST_DESARMADO) set_state(ST_DESARMADO);
		else send_status();
		break;

		case CMD_SILENCE:
		if (state == ST_ALARMA) set_state(ST_SILENCIADA);   //sigue armado, sin sirena 
		else send_status();
		break;

		case CMD_AUTO:
		mode = MODE_AUTO;
		lcd_dirty = 1;
		send_status();
		break;

		case CMD_MANUAL:
		mode = MODE_MANUAL;
		lcd_dirty = 1;
		send_status();
		break;

		case CMD_FORCE:
		if (state != ST_ALARMA) {
			zone = ZONE_REMOTE;
			set_state(ST_ALARMA);
			} else {
			send_status();
		}
		break;

		case CMD_STATUS:
		default:
		send_status();
		break;
	}
}

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
//  PROCESAMIENTO DE TRAMAS RECIBIDAS 
static void process_line(const char *l) {
	cmd_t cmd;

	if      (!strcmp(l, "ARMAR"))     cmd = CMD_ARM;
	else if (!strcmp(l, "DESARMAR"))  cmd = CMD_DISARM;
	else if (!strcmp(l, "SILENCIAR")) cmd = CMD_SILENCE;
	else if (!strcmp(l, "AUTO"))      cmd = CMD_AUTO;
	else if (!strcmp(l, "MANUAL"))    cmd = CMD_MANUAL;
	else if (!strcmp(l, "FORZAR"))    cmd = CMD_FORCE;
	else if (!strcmp(l, "ESTADO"))    cmd = CMD_STATUS;
	else {
		uart_print("ERR:COMANDO\r\n");
		return;
	}

	strncpy(last_rx, l, 3);         // ultimos 3 caracteres recibidos para el LCD 
	last_rx[3] = '\0';
	uart_print("ACK:");
	uart_print(l);
	uart_print("\r\n");
	handle_command(cmd);
	lcd_dirty = 1;
}

static void bt_poll(void) {
	while (rx_tail != rx_head) {
		char c = (char)rx_buf[rx_tail];
		rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

		if (c == '\r' || c == '\n') {
			if (line_len > 0) {
				line[line_len] = '\0';
				process_line(line);
				line_len = 0;
			}
			} else if (line_len < LINE_MAX_LEN - 1) {
			line[line_len++] = (char)toupper((unsigned char)c);
		}
	}
}

//   SENSORES Y BOTONES 
static uint8_t debounce(uint8_t active, uint8_t *cnt) {
	// Devuelve 1 solo en el flanco de validacion (rising edge confirmado) 
	if (!active) {
		*cnt = 0;
		return 0;
	}
	if (*cnt < 255) (*cnt)++;
	return (*cnt == SENSOR_CONFIRM);
}

static uint8_t pir_active(void) {
	#if PIR_ACTIVE_HIGH
	return (PIND & (1 << PIR_PIN)) != 0;
	#else
	return (PIND & (1 << PIR_PIN)) == 0;
	#endif
}

static uint8_t ldr_open(void) {
	uint16_t v = adc_read(LDR_CH);
	#if LDR_OPEN_WHEN_BRIGHT
	return v > LDR_THRESHOLD;
	#else
	return v < LDR_THRESHOLD;
	#endif
}

static void sensors_poll(void) {
	if (state == ST_ARMADO) {
		if (debounce(ldr_open(), &ldr_cnt)) on_event(ZONE_WINDOW);
		if (state == ST_ARMADO && debounce(pir_active(), &pir_cnt)) on_event(ZONE_MOTION);
		} else {
		ldr_cnt = 0;
		pir_cnt = 0;
	}
}

static uint8_t edge(uint8_t now, uint8_t *prev) {
	uint8_t e = now && !*prev;
	*prev = now;
	return e;
}

static void buttons_poll(void) {
	uint8_t arm  = !(PIND & (1 << BTN_ARM_PIN));
	uint8_t sil  = !(PIND & (1 << BTN_SIL_PIN));
	uint8_t mod  = !(PINB & (1 << BTN_MODE_PIN));

	if (edge(arm, &prev_arm)) {
		handle_command(state == ST_DESARMADO ? CMD_ARM : CMD_DISARM);
	}
	if (edge(sil, &prev_sil)) {
		handle_command(CMD_SILENCE);
	}
	if (edge(mod, &prev_mode)) {
		handle_command(mode == MODE_AUTO ? CMD_MANUAL : CMD_AUTO);
	}
}

// SALIDAS (LEDs y buzzer) 
static void update_outputs(uint32_t now) {
	uint8_t blink = (now / 250) & 1;

	switch (state) {
		case ST_DESARMADO:
		PORTD &= ~((1 << LED_G_PIN) | (1 << LED_R_PIN) | (1 << BUZ_PIN));
		break;

		case ST_ARMADO:
		PORTD |=  (1 << LED_G_PIN);
		PORTD &= ~((1 << LED_R_PIN) | (1 << BUZ_PIN));
		break;

		case ST_ALARMA:
		PORTD &= ~(1 << LED_G_PIN);
		PORTD |=  (1 << BUZ_PIN);
		if (blink) PORTD |= (1 << LED_R_PIN);
		else       PORTD &= ~(1 << LED_R_PIN);
		break;

		case ST_SILENCIADA:
		PORTD |=  (1 << LED_G_PIN) | (1 << LED_R_PIN);   // armado + alarma memorizada 
		PORTD &= ~(1 << BUZ_PIN);
		break;
	}
}

// LCD 
#if USE_LCD
static void lcd_refresh(void) {
	char b[20];

	snprintf(b, sizeof(b), "%-7s %-8s", state_lcd[state], mode == MODE_AUTO ? "AUTO" : "MANUAL");
	twi_lcd_cmd(0x80);
	twi_lcd_msg(b);

	snprintf(b, sizeof(b), "Z:%-7s RX:%-3s", zone_lcd[zone], last_rx);
	twi_lcd_cmd(0xC0);
	twi_lcd_msg(b);
}
#endif

//  INICIALIZACION 
static void io_init(void) {
	// Salidas: LED verde, LED rojo, buzzer 
	DDRD |= (1 << LED_G_PIN) | (1 << LED_R_PIN) | (1 << BUZ_PIN);
	PORTD &= ~((1 << LED_G_PIN) | (1 << LED_R_PIN) | (1 << BUZ_PIN));

	// Entradas: PIR y botones 
	DDRD &= ~((1 << PIR_PIN) | (1 << BTN_ARM_PIN) | (1 << BTN_SIL_PIN));
	PORTD |= (1 << BTN_ARM_PIN) | (1 << BTN_SIL_PIN);
	#if !PIR_ACTIVE_HIGH
	PORTD |= (1 << PIR_PIN);
	#endif
	DDRB &= ~(1 << BTN_MODE_PIN);
	PORTB |= (1 << BTN_MODE_PIN);
}
// PROGRAMA PRINCIPAL 
int main(void) {
	io_init();
	adc_init();
	timer0_init();

	uart_init(MY_UBRR);
	UCSR0B |= (1 << RXCIE0);        // habilitar interrupcion de recepcion

	#if USE_LCD
	twi_init();                     // ojo: twi_init pone PC0/PC1 como salida -> LDR en PC2 
	twi_lcd_init();
	#endif

	sei();

	uart_print("SISTEMA:ALARMA LISTO\r\n");
	send_status();

	uint32_t t_sensor = 0, t_status = 0, t_lcd = 0;

	while (1) {
		uint32_t now = millis();

		bt_poll();

		if (now - t_sensor >= SENSOR_PERIOD_MS) {
			t_sensor = now;
			sensors_poll();
			buttons_poll();
		}

		update_outputs(now);

		if (now - t_status >= STATUS_PERIOD_MS) {
			t_status = now;
			send_status();
		}

		#if USE_LCD
		if (lcd_dirty && (now - t_lcd >= LCD_MIN_PERIOD_MS)) {
			t_lcd = now;
			lcd_dirty = 0;
			lcd_refresh();
		}
		#endif
	}

	return 0;
}

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "UART.h"
#include "SPI.h"
#include "RC522.h"
#include "twi.h"

#define PCF8574 0x27  
#include "twi_lcd.h"

#define BAUD_PRESCALE 103
#define LED_VERDE PC0
#define LED_ROJO PC1
#define BUZZER PC2
#define ROW_MASK 0xF0
#define COL_C1 PD2
#define COL_C2 PD3
#define COL_C3 PC3 
#define COL_C4 PB0

typedef struct { uint8_t uid[4]; uint8_t pin[4]; uint32_t saldo; } cuenta_t;
#define N_CUENTAS 3

static cuenta_t EEMEM ee_cuentas[N_CUENTAS] = {
	{{0xA1,0xB2,0xC3,0xD4},{'1','2','3','4'},5000},
	{{0x11,0x22,0x33,0x44},{'5','6','7','8'},3000},
	{{0xDE,0xAD,0xBE,0xEF},{'9','9','9','9'},1000}
};

static cuenta_t cuentas[N_CUENTAS];

static uint8_t cuenta_actual=0xFF;

typedef enum { ST_ESPERA_TARJETA,ST_PIN,ST_MENU,ST_MONTO_RETIRO,
ST_MONTO_DEPOSITO,ST_MOSTRAR_SALDO,ST_BLOQUEO } estado_t;

static estado_t estado=ST_ESPERA_TARJETA;
static char pin_buf[5];

static uint8_t pin_idx=0,intentos=0;

static uint32_t monto=0;

static const char teclas[4][4]={{'1','2','3','A'},{'4','5','6','B'},
{'7','8','9','C'},{'*','0','#','D'}};

/* ---------- Prototipos de funciones ---------- */
static void keypad_init(void);
static uint8_t col_pressed(uint8_t c);
static char keypad_scan(void);
static void indicadores_init(void);
static void ok(void);
static void error_led(void);
static void buzz(void);
static void cargar_cuentas(void);
static void guardar_saldo(uint8_t idx);
static uint8_t detectar_tarjeta(uint8_t *uid);
static void lcd_num(uint32_t numero);
static void lcd_goto(uint8_t col,uint8_t fila);
static void lcd_show(const char *l1,const char *l2);
static uint32_t leer_monto(const char *titulo);


// Implementacion de funciones 

static void keypad_init(void) {
	DDRD |= ROW_MASK; PORTD |= ROW_MASK;
	DDRD &= ~((1<<COL_C1)|(1<<COL_C2));
	PORTD |= (1<<COL_C1)|(1<<COL_C2);
	DDRC &= ~(1<<COL_C3); PORTC |= (1<<COL_C3);
	DDRB &= ~(1<<COL_C4); PORTB |= (1<<COL_C4);
}

static uint8_t col_pressed(uint8_t c) {
	switch(c) {
		case 0: return !(PIND & (1<<COL_C1));
		case 1: return !(PIND & (1<<COL_C2));
		case 2: return !(PINC & (1<<COL_C3));
		default: return !(PINB & (1<<COL_C4));
	}
}
static char keypad_scan(void) {
	for(uint8_t f=0;f<4;f++) {
		PORTD |= ROW_MASK;
		PORTD &= ~(1<<(4+f));
		_delay_us(5);
		for(uint8_t c=0;c<4;c++) {
			if(col_pressed(c)) {
				_delay_ms(25);
				if(!col_pressed(c)) continue;
				while(col_pressed(c));
				PORTD |= ROW_MASK;
				return teclas[f][c];
			}
		}
	}
	PORTD |= ROW_MASK;
	return 0;
}
static void indicadores_init(void) {
	DDRC |= (1<<LED_VERDE)|(1<<LED_ROJO)|(1<<BUZZER);
	PORTC &= ~((1<<LED_VERDE)|(1<<LED_ROJO)|(1<<BUZZER));
}

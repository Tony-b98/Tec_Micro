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

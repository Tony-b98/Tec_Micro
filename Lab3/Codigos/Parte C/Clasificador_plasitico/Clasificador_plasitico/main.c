#define F_CPU 16000000UL
#define PCF8574 0x27  
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "twi_lcd.h"

#define N_COLORES 4
#define N_LEDS 1               // Cantidad REAL de LEDs de la tira 
#define WS2812_HABILITADA 1    // Solo afecta al modo WS2812 
#define WS_DDR_REG DDRD
#define WS_PORT_REG PORTD
#define WS_BIT (1 << PD6)
#define ADC_PROM 16
#define BARRIDOS_CAL 16
#define ESTABILIZACION_MS 180  // Aumentar si la LDR aun no se estabiliza 
#define MARGEN_ADC 8           // Margen inicial ajustable; NO valor medido 
#define PAGINA_MS 1800
#define SERVO_RECORRIDO_GRADOS 180
#define SERVO_PULSO_MIN_US 1000UL
#define SERVO_PULSO_MAX_US 2000UL
#define EEPROM_MAGIC 0xC404
#define EEPROM_VERSION 1

// 1 = LED RGB de 4 patas (prueba en mesa). 0 = WS2812 (laboratorio). 
#define MODO_RGB 1
#if MODO_RGB
#define BIT_R (1 << PD3)   // Rojo  -> D3 
#define BIT_G (1 << PD5)   // Verde -> D5
#define BIT_B (1 << PD6)   // Azul  -> D6 
#define RGB_MASK (BIT_R | BIT_G | BIT_B)
#endif

#if SERVO_RECORRIDO_GRADOS != 180 && SERVO_RECORRIDO_GRADOS != 360
#error Usar 180 o 360 grados de recorrido POSICIONAL
#endif

// Orden de derecha a izquierda del semicirculo.
typedef struct { char nombre[9]; uint8_t r,g,b; } color_t;
static const color_t COLORES[N_COLORES] = {
	{"NARANJA", 255, 100,   0},
	{"BLANCO",  255, 255, 255},
	{"VIOLETA", 150,   0, 255},
	{"MARRON",  120,  55,  20}
};
// SG90 posicional, un destino por color.
static const uint16_t ANGULOS[N_COLORES] = {22, 67, 112, 157};

// Calibracion guardada en EEPROM
typedef struct { uint16_t referencia, minimo, maximo; } rango_t;
typedef struct {
	uint16_t magic;
	uint8_t version, validos;
	uint16_t firma;
	rango_t color[N_COLORES][N_COLORES]; // [plastico][iluminacion]
	uint16_t checksum;
} calibracion_t;
static calibracion_t cal;
static calibracion_t EEMEM cal_eeprom;
static uint16_t lectura[N_COLORES];
static uint16_t angulo_aplicado;
static int8_t detectado = -1; // -1 = fuera de rango, -2 = ambiguo
static uint8_t hay_resultado, pagina;
static volatile uint8_t ocupado, pendiente;
static volatile char comando;
typedef enum {REPOSO, MIDIENDO, CLASIFICANDO, MOSTRANDO} estado_t;
static estado_t estado = REPOSO;

// PROTOTIPOS

// UART
static void uart_putc(char c);
static void uart_puts(const char *s);
static void uart_init(void);
static char tomar_comando(void);

// LCD
static void lcd_nibble_inicio(uint8_t nibble);
static void lcd_inicio_corregido(void);
static void lcd_clear_safe(void);
static void lcd_linea(uint8_t fila, const char *texto);
static void pantalla(const char *a, const char *b);

// ADC
static void adc_init(void);
static uint16_t adc_promedio(void);

// WS2812
#if !MODO_RGB
extern void ws_send_frame(const uint8_t *frame, uint8_t cantidad,
uint8_t alto, uint8_t bajo);
#endif
static void ws_show(uint8_t r, uint8_t g, uint8_t b);
static void ws_apagar(void);
static void luz_color(uint8_t i);

// Servo
static void servo_angulo(uint16_t grados);
static void servo_init(void);

// EEPROM
static uint16_t hash_bytes(const void *p, uint16_t n);
static uint16_t firma_config(void);
static void guardar(void);
static void cargar(void);

// Menu y clasificacion
static void ayuda(void);
static void listar(void);
static void barrido(uint8_t numero_cal);
static void calibrar(uint8_t i);
static int8_t clasificar(const calibracion_t *c, const uint16_t v[N_COLORES]);
static void reportar(void);
static void mostrar_pagina(void);
static void diagnostico_adc(void);
static void prueba_ws2812(void);

// IMPLEMENTACION DE FUNCIONES

static void uart_putc(char c) {
	while (!(UCSR0A & (1 << UDRE0))) {}
	UDR0 = c;
}
static void uart_puts(const char *s) {
	while (*s) uart_putc(*s++);
}

static void uart_init(void) {
	UCSR0A = 0; UBRR0 = 103;
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
ISR(USART_RX_vect) {
	char c = UDR0;
	if (!ocupado && !pendiente &&
	((c >= '1' && c <= '4') || c=='m' || c=='M' ||
	c=='p' || c=='P' || c=='h' || c=='H' || c=='x' || c=='X' ||
	c=='a' || c=='A' || c=='t' || c=='T')) {
		comando = c; pendiente = 1;
	}
}
static char tomar_comando(void) {
	char c = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (pendiente) { c = comando; pendiente = 0; ocupado = 1; }
	}
	return c;
}

static void lcd_nibble_inicio(uint8_t nibble) {
	uint8_t dato = (nibble & 0xF0) | 0x08;
	PCF8574_write(dato);
	_delay_us(2);
	PCF8574_write(dato | 0x04);
	_delay_us(5);
	PCF8574_write(dato);
	_delay_us(100);
}

static void lcd_clear_safe(void) {
	twi_lcd_clear();
	_delay_ms(3);
}

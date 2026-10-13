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
static void lcd_inicio_corregido(void) {
	_delay_ms(100);               /* Estabilizacion de alimentacion */
	twi_init();
	_delay_ms(50);
	
	lcd_nibble_inicio(0x30);
	_delay_ms(5);
	lcd_nibble_inicio(0x30);
	_delay_us(200);
	lcd_nibble_inicio(0x30);
	_delay_us(200);
	lcd_nibble_inicio(0x20);      /* Pasar a modo 4 bits */
	_delay_us(200);

	twi_lcd_cmd(0x28);            /* 4 bits, 2 lineas, matriz 5x8 */
	_delay_ms(2);
	twi_lcd_cmd(0x08);            /* Display apagado */
	_delay_ms(2);
	lcd_clear_safe();
	twi_lcd_cmd(0x06);            /* Incrementar cursor sin desplazar */
	_delay_ms(2);
	twi_lcd_cmd(0x0C);            /* Display encendido sin cursor */
	_delay_ms(2);
	twi_lcd_cmd(0x80);            /* Primera fila, columna 1 */
	_delay_ms(2);
}

static void lcd_linea(uint8_t fila, const char *texto) {
	char linea[17];
	uint8_t n = 0;
	while (n < 16 && texto[n]) { linea[n] = texto[n]; ++n; }
	while (n < 16) linea[n++] = ' ';
	linea[16] = 0;
	twi_lcd_cmd(fila ? 0xC0 : 0x80);
	twi_lcd_msg(linea);
}
static void pantalla(const char *a, const char *b) {
	lcd_linea(0,a); lcd_linea(1,b);
}
static void adc_init(void) {
	DDRC &= ~(1 << PC0); PORTC &= ~(1 << PC0);
	DIDR0 |= (1 << ADC0D);
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
static uint16_t adc_promedio(void) {
	uint32_t suma = 0;
	for (uint8_t i=0; i<ADC_PROM; ++i) {
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC)) {}
		suma += ADC;
	}
	return (uint16_t)(suma / ADC_PROM);
}
/* SALIDA DE LUZ: dos implementaciones intercambiables.
 * Todo el programa solo llama a ws_show() y ws_apagar().
 */ 
#if MODO_RGB
/* MODO PRUEBA: LED RGB de 4 patas, CATTODO COMUN */
static void ws_show(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t v = 0;
    if (r) v |= BIT_R;
    if (g) v |= BIT_G;
    if (b) v |= BIT_B;
    PORTD = (PORTD & ~RGB_MASK) | v;   /* conserva PD0/PD1 de la UART */
}
static void ws_apagar(void) { ws_show(0, 0, 0); }

#else
/*MODO LABORATORIO: WS2812 ORIGINAL*/
#if N_LEDS < 1 || N_LEDS > 85
#error N_LEDS debe estar entre 1 y 85
#endif
#define WS_HEAD "out 0x0b, r20\n\t" \
                "nop\n\tnop\n\tnop\n\t" \
                "sbrs r19, 7\n\t" \
                "out 0x0b, r18\n\t" \
                "nop\n\tnop\n\tnop\n\tnop\n\t" \
                "out 0x0b, r18\n\t"
#define WS_FULL WS_HEAD "lsl r19\n\t" \
                "nop\n\tnop\n\tnop\n\tnop\n\t" \
                "nop\n\tnop\n\tnop\n\tnop\n\t"

/* Funcion en ensamblador definida en este MISMO main.c.*/
__asm__(
    ".text\n"
    ".global ws_send_frame\n"
    ".type ws_send_frame, @function\n"
    "ws_send_frame:\n\t"
    "movw r30, r24\n\t"
    "tst r22\n\t"
    "brne 3f\n\t"
    "ret\n\t"
    "3:\n\t"
    "ld r19, Z+\n\t"
    "1:\n\t"
    WS_FULL WS_FULL WS_FULL WS_FULL WS_FULL WS_FULL WS_FULL
    WS_HEAD
    "dec r22\n\t"
    "breq 2f\n\t"
    "ld r19, Z+\n\t"
    "nop\n\tnop\n\tnop\n\t"
    "rjmp 1b\n\t"
    "2:\n\t"
    "ret\n"
    ".size ws_send_frame, .-ws_send_frame\n"
);

static void ws_show(uint8_t r, uint8_t g, uint8_t b) {
#if WS2812_HABILITADA
    uint8_t frame[N_LEDS * 3];
    for (uint8_t i = 0; i < N_LEDS; ++i) {
        frame[3*i] = g; frame[3*i+1] = r; frame[3*i+2] = b;
    }
    uint8_t saved = SREG;
    cli();
    uint8_t hi = PORTD | WS_BIT, lo = PORTD & ~WS_BIT;
    ws_send_frame(frame, (uint8_t)sizeof(frame), hi, lo);
    SREG = saved;
    _delay_us(300);
#else
    (void)r; (void)g; (void)b;
#endif
}
static void ws_apagar(void) { ws_show(0, 0, 0); }
#endif /* MODO_RGB */

static void luz_color(uint8_t i) {
    ws_show(COLORES[i].r, COLORES[i].g, COLORES[i].b);
}
static void servo_angulo(uint16_t grados) {
    if (grados > SERVO_RECORRIDO_GRADOS) grados = SERVO_RECORRIDO_GRADOS;
    uint32_t us = SERVO_PULSO_MIN_US +
        (SERVO_PULSO_MAX_US-SERVO_PULSO_MIN_US)*grados/SERVO_RECORRIDO_GRADOS;
    /* 16MHz/64=250kHz: cada tick dura 4us. */
    OCR1A = (uint16_t)((us+2)/4);
    angulo_aplicado = grados;
}
static void servo_init(void) {
    DDRB |= (1 << PB1);
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    ICR1 = 4999;
    servo_angulo(SERVO_RECORRIDO_GRADOS/2);
}
static uint16_t hash_bytes(const void *p, uint16_t n) {
    const uint8_t *b = p;
    uint16_t h = 0xA561;
    while (n--) h = (uint16_t)((h << 5) | (h >> 11)) ^ *b++;
    return h;
}
static uint16_t firma_config(void) {
    return hash_bytes(COLORES,sizeof COLORES) ^ (uint16_t)N_LEDS ^
           ((uint16_t)MARGEN_ADC << 8);
}
static void guardar(void) {
    cal.magic = EEPROM_MAGIC; cal.version = EEPROM_VERSION;
    cal.firma = firma_config();
    cal.checksum = hash_bytes(&cal,offsetof(calibracion_t,checksum));
    eeprom_update_block(&cal,&cal_eeprom,sizeof cal);
}
static void cargar(void) {
    uint8_t ok = 1;
    eeprom_read_block(&cal,&cal_eeprom,sizeof cal);
    if (cal.magic != EEPROM_MAGIC || cal.version != EEPROM_VERSION ||
        cal.firma != firma_config() || (cal.validos & 0xF0) ||
        cal.checksum != hash_bytes(&cal,offsetof(calibracion_t,checksum))) ok=0;
    for (uint8_t i=0; i<N_COLORES && ok; ++i) {
        if (!(cal.validos & (1 << i))) continue;
        for (uint8_t j=0; j<N_COLORES; ++j) {
            const rango_t *r = &cal.color[i][j];
            if (r->minimo > r->referencia || r->referencia > r->maximo ||
                r->maximo > 1023) ok=0;
        }
    }
  if (!ok) memset(&cal,0,sizeof cal);
  }
  static void ayuda(void) {
	  char msg[64];
	  uart_puts("\r\n=== CLASIFICADOR 4 PLASTICOS / ADC REAL ===\r\n");
	  for (uint8_t i=0; i<N_COLORES; ++i) {
		  snprintf(msg,sizeof msg,"%u: calibrar %-8s | servo %u grados\r\n",
		  (unsigned)(i+1),COLORES[i].nombre,(unsigned)ANGULOS[i]);
		  uart_puts(msg);
	  }
	  uart_puts("Colocar la muestra ANTES de pulsar su numero.\r\n"
	  "m: medir | p: patrones/rangos | h: ayuda | x: borrar\r\n"
	  "a: lectura ADC | t: prueba WS2812\r\n"
	  "Esperar el fin de cada operacion antes de enviar otra tecla.\r\n");
	  #if SERVO_RECORRIDO_GRADOS == 180
	  uart_puts("Servo: 180 grados, usar escala SEMICIRCULAR.\r\n");
	  #else
	  uart_puts("Servo: requiere control POSICIONAL de 360 grados.\r\n");
	  #endif
  }
  static void listar(void) {
	  char msg[80];
	  for (uint8_t i=0; i<N_COLORES; ++i) {
		  snprintf(msg,sizeof msg,"\r\n%u %s: %s\r\n",(unsigned)(i+1),
		  COLORES[i].nombre,(cal.validos & (1 << i)) ? "CALIBRADO" : "SIN CALIBRAR");
		  uart_puts(msg);
		  if (!(cal.validos & (1 << i))) continue;
		  for (uint8_t j=0; j<N_COLORES; ++j) {
			  const rango_t *r=&cal.color[i][j];
			  snprintf(msg,sizeof msg,"  Luz %-8s Ref=%u Rango=[%u,%u]\r\n",
			  COLORES[j].nombre,(unsigned)r->referencia,
			  (unsigned)r->minimo,(unsigned)r->maximo);
			  uart_puts(msg);
		  }
	  }
  }
/* Cada lectura queda asociada a la luz que estaba encendida al medir. */
static void barrido(uint8_t numero_cal) {
    char a[17],b[17];
    for (uint8_t j=0; j<N_COLORES; ++j) {
        if (numero_cal) snprintf(a,sizeof a,"Calibrando %u/%u",
                    (unsigned)numero_cal,(unsigned)BARRIDOS_CAL);
        else strcpy(a,"Midiendo...");
        snprintf(b,sizeof b,"Luz:%s",COLORES[j].nombre);
        pantalla(a,b);
        luz_color(j);
        _delay_ms(ESTABILIZACION_MS);
        lectura[j]=adc_promedio();
    }
}
/* Los intervalos se obtienen de varias mediciones del MISMO plastico.
 * El margen amplifica min/max para absorber pequenas variaciones.
 * En la calibracion se debe conservar la posicion y variar ligeramente
 * la zona de la muestra si eso representa el uso real.
 */
static void calibrar(uint8_t i) {
    uint32_t suma[N_COLORES]={0};
    uint16_t minimo[N_COLORES]={1023,1023,1023,1023};
    uint16_t maximo[N_COLORES]={0};
    char msg[64];
    hay_resultado=0;
    snprintf(msg,sizeof msg,"\r\nCalibrando %s. Mantener muestra colocada.\r\n",COLORES[i].nombre);
    uart_puts(msg);
    for (uint8_t n=1; n<=BARRIDOS_CAL; ++n) {
        barrido(n);
        for (uint8_t j=0; j<N_COLORES; ++j) {
            uint16_t v=lectura[j];
            suma[j]+=v;
            if (v<minimo[j]) minimo[j]=v;
            if (v>maximo[j]) maximo[j]=v;
        }
    }

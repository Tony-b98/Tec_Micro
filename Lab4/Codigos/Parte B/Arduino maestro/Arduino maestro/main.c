#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#define PCF8574       0x27
#define TEMP_LIMITE   28
#define TEMP_REARME   26
#define UMBRAL_LDR    500

#define CMD_RUN       0x01
#define CMD_SPEED     0x02
#define CMD_PISTON    0x03
#define CMD_ALARM     0x04

static uint8_t en_marcha = 0;
static uint8_t alarma = 0;

static uint16_t piezas = 0;

static uint8_t temp_c = 0;
static uint8_t humedad = 0;
static uint8_t temp_valida = 0;
static uint8_t velocidad = 0;


  // COMUNICACION SPI - MAESTRO
   
   static void spi_init(void) {

    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
    DDRB &= ~(1 << PB4);

    PORTB |= (1 << PB2);

    SPCR = (1 << SPE) |
           (1 << MSTR) |
           (1 << SPR1) |
           (1 << SPR0);
}

static void spi_tx(uint8_t dato) {

    SPDR = dato;

    while (!(SPSR & (1 << SPIF))) {}

    (void)SPDR;
}

static void enviar(uint8_t cmd, uint8_t valor) {

    PORTB &= ~(1 << PB2);

    _delay_us(20);

    spi_tx(cmd);

    _delay_us(100);

    spi_tx(valor);

    _delay_us(20);

    PORTB |= (1 << PB2);
}
/* ==================================================
   CONVERSOR ANALOGICO DIGITAL
   ================================================== */

static void adc_init(void) {

    ADMUX = (1 << REFS0);

    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) |
             (1 << ADPS1) |
             (1 << ADPS0);
}

static uint16_t adc_leer(uint8_t canal) {

    ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);

    _delay_us(30);

    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC)) {}

    return ADC;
}

// LCD I2C - LIBRERIAS DEL DOCENTE

#include "twi_lcd.h"
static void lcd_cmd(uint8_t c) {

    twi_lcd_cmd(c);

    if (c == 1 || c == 2) {
        _delay_ms(2);
    }
}

static void lcd_char(char c) {
    twi_lcd_dwr((unsigned char)c);
}

static void lcd_texto(const char *s) {
    twi_lcd_msg(s);
}

static void lcd_pos(uint8_t fila, uint8_t col) {

    lcd_cmd((uint8_t)(0x80 |
            (fila ? 0x40 : 0) | col));
}

static void lcd_init(void) {
    twi_lcd_init();
}

//ACTUALIZACION DE PANTALLA LCD
static void lcd_actualizar(void) {

    char linea[17];
    uint8_t n;

    lcd_pos(0, 0);

    if (alarma) {

        // Primera fila:

        snprintf(linea, sizeof(linea),
                 "EMERGENCIA T:%2uC", temp_c);

        lcd_texto(linea);

        n = 0;

        while (linea[n]) n++;

        while (n++ < 16) {
            lcd_char(' ');
        }

        // Segunda fila:
        
		lcd_pos(1, 0);
		lcd_texto("MOTOR DETENIDO  ");
    }
    else {

        // Primera fila:
           
        snprintf(linea, sizeof(linea),
                 "T:%2uC V:%3u %s",
                 temp_c,
                 velocidad,
                 en_marcha ? "ON" : "OFF");

        lcd_texto(linea);

        n = 0;

        while (linea[n]) n++;

        while (n++ < 16) {
            lcd_char(' ');
        }

        // Segunda fila:

        lcd_pos(1, 0);

        snprintf(linea, sizeof(linea),
                 "PIEZAS:%5u %s",
                 piezas,
                 temp_valida ? "OK" : "ERR");

        lcd_texto(linea);

        n = 0;

        while (linea[n]) n++;

        while (n++ < 16) {
            lcd_char(' ');
        }
    }
}

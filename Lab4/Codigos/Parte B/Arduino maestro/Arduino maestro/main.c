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

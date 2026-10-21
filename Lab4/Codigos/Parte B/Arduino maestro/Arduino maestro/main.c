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
//CONVERSOR ANALOGICO DIGITAL 

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
//SENSOR DE TEMPERATURA DHT11
#define DHT_BIT PD2

static uint8_t esperar_dht(uint8_t alto,
                           uint16_t limite_us) {

    while (limite_us--) {

        uint8_t estado =
            (PIND & (1 << DHT_BIT)) ? 1 : 0;

        if (estado == alto) {
            return 1;
        }

        _delay_us(1);
    }

    return 0;
}

static uint8_t dht11_leer(uint8_t *temperatura,
                          uint8_t *hum) {

    uint8_t datos[5] = {0};

    DDRD |= (1 << DHT_BIT);

    PORTD &= ~(1 << DHT_BIT);

    _delay_ms(20);

    DDRD &= ~(1 << DHT_BIT);

    PORTD |= (1 << DHT_BIT);

    _delay_us(30);

    if (!esperar_dht(0, 120)) return 0;
    if (!esperar_dht(1, 120)) return 0;
    if (!esperar_dht(0, 120)) return 0;

    for (uint8_t i = 0; i < 40; i++) {

        if (!esperar_dht(1, 90)) return 0;

        _delay_us(40);

        datos[i / 8] <<= 1;

        if (PIND & (1 << DHT_BIT)) {
            datos[i / 8] |= 1;
        }

        if (!esperar_dht(0, 110)) return 0;
    }

    if ((uint8_t)(datos[0] +
                  datos[1] +
                  datos[2] +
                  datos[3]) != datos[4]) {
        return 0;
    }

    *hum = datos[0];

    *temperatura = datos[2];

    return 1;
}

// PROGRAMA PRINCIPAL

int main(void) {

    DDRD &= ~(1 << PD3);

    PORTD |= (1 << PD3);

    spi_init();
    adc_init();
    twi_init();
    lcd_init();

    lcd_pos(0, 0);
    lcd_texto("LAB4 SPI MAESTRO");

    lcd_pos(1, 0);
    lcd_texto("Inicializando...");

    _delay_ms(700);

    enviar(CMD_RUN, 0);
    enviar(CMD_ALARM, 0);

    uint8_t btn_previo = 1;

    uint8_t ldr_previo_oscuro = 0;
    uint8_t bloqueo_pieza = 0;

    uint8_t ticks_100ms = 0;
    uint8_t ticks_temp = 0;
    uint8_t ticks_lcd = 0;

    uint8_t velocidad_anterior = 255;

    while (1) {

        
          // BOTON START / STOP

        uint8_t btn =
            (PIND & (1 << PD3)) ? 1 : 0;

        if (!btn && btn_previo) {

            _delay_ms(25);

            if (!(PIND & (1 << PD3)) && !alarma) {

                en_marcha = !en_marcha;

                enviar(CMD_RUN, en_marcha);
            }
        }

        btn_previo = btn;

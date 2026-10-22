#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>

#define CMD_RUN       0x01
#define CMD_SPEED     0x02
#define CMD_PISTON    0x03
#define CMD_ALARM     0x04
#define SERVO_REPOSO  2000 // 1.0 ms en Timer1 /8 
#define SERVO_EXT     4000 // 2.0 ms en Timer1 /8 
#define PISTON_MS     500

static volatile uint8_t spi_estado = 0;
static volatile uint8_t spi_comando = 0;
static volatile uint8_t cmd_pendiente = 0;
static volatile uint8_t val_pendiente = 0;
static volatile uint8_t comando_listo = 0;
static volatile uint32_t milisegundos = 0;

ISR(SPI_STC_vect) {
    uint8_t recibido = SPDR;
    if (spi_estado == 0) {
        spi_comando = recibido;
        spi_estado = 1;
    } else {
        // Un solo frame pendiente; maestro espera entre envios. 
        cmd_pendiente = spi_comando;
        val_pendiente = recibido;
        comando_listo = 1;
        spi_estado = 0;
    }
    SPDR = 0x00;
}

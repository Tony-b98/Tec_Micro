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
ISR(TIMER2_COMPA_vect) { milisegundos++; }

static void timer2_init(void) {
	TCCR2A = (1 << WGM21);
	TCCR2B = (1 << CS22); //64 
	OCR2A = 249;          //1 ms 
	TIMSK2 = (1 << OCIE2A);
}
static uint32_t tiempo_ms(void) {
	uint32_t t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = milisegundos; }
	return t;
}
static void spi_slave_init(void) {
	DDRB &= ~((1 << PB2) | (1 << PB3) | (1 << PB5)); // SS MOSI SCK entradas 
	DDRB |= (1 << PB4); // MISO salida 
	SPCR = (1 << SPE) | (1 << SPIE);
	SPDR = 0;
}
static void motor_init(void) {
	DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5)
	| (1 << PD6) | (1 << PD7);
	PORTD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4)
	| (1 << PD5) | (1 << PD7));
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01); // Fast PWM D6 OC0A ~7.8 kHz
	OCR0A = 0;
}
static void servo_init(void) {
	DDRB |= (1 << PB1); //D9 OC1A 
	// Fast PWM Timer1 modo 14, TOP = ICR1, /8 => 2 MHz 
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	ICR1 = 39999;    // 20 ms, 50 Hz 
	OCR1A = SERVO_REPOSO;
}

int main(void) {
	motor_init(); servo_init(); spi_slave_init(); timer2_init();
	sei();
	uint8_t marcha = 0, falla = 0, pwm_deseado = 0;
	uint8_t piston_activo = 0;
	uint32_t piston_inicio = 0;

	while (1) {
		uint8_t cmd = 0, valor = 0, recibido = 0;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			if (comando_listo) {
				cmd = cmd_pendiente;
				valor = val_pendiente;
				comando_listo = 0;
				recibido = 1;
			}
		}
		if (recibido) {
			switch (cmd) {
				case CMD_RUN:
				if (!falla) marcha = (valor != 0);
				break;
				case CMD_SPEED:
				pwm_deseado = valor;
				break;
				case CMD_PISTON:
				if (valor && marcha && !falla) {
					OCR1A = SERVO_EXT;
					piston_inicio = tiempo_ms();
					piston_activo = 1;
				}
				break;
				case CMD_ALARM:
				if (valor) {
					falla = 1;
					marcha = 0;
					piston_activo = 0;
					OCR1A = SERVO_REPOSO;
					} else {
					falla = 0;
					marcha = 0;
				}
				break;
				default: break;
			}
		}
     if (piston_activo && (uint32_t)(tiempo_ms() - piston_inicio) >= PISTON_MS) {
	     OCR1A = SERVO_REPOSO;
	     piston_activo = 0;
     }

     if (marcha && !falla) {
	     PORTD |= (1 << PD4); /* IN1 alto */
	     PORTD &= ~(1 << PD5); /* IN2 bajo */
	     OCR0A = pwm_deseado;
	     } else {
	     OCR0A = 0;
	     PORTD &= ~((1 << PD4) | (1 << PD5));
     }
     if (marcha && !falla) PORTD |= (1 << PD2);
     else PORTD &= ~(1 << PD2);
     if (falla) PORTD |= (1 << PD3) | (1 << PD7);
     else PORTD &= ~((1 << PD3) | (1 << PD7));
     }
     }

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

#define LED_PIN   PB5   // D13
#define TEST_PIN  PB1   // D9 / OC1A
#define BOTON_PIN PD2   // D2 / INT0

#define N_FRECUENCIAS 5U

// Bits CLKPS3..0 del registro CLKPR para cada division.
static const uint8_t clkpr_bits[N_FRECUENCIAS] = {
	0,                                              // /1   : 16 MHz
	(1 << CLKPS0),                                  // /2   :  8 MHz
	(1 << CLKPS1) | (1 << CLKPS0),                  // /8   :  2 MHz
	(1 << CLKPS2) | (1 << CLKPS1),                  // /64  : 250 kHz
	(1 << CLKPS2) | (1 << CLKPS1) | (1 << CLKPS0)   // /128 : 125 kHz
};

volatile uint8_t boton = 0;

ISR(INT0_vect)
{
	EIMSK &= ~(1 << INT0);    // evita reentradas mientras se pulsa
	boton = 1;
}

// Secuencia protegida del datasheet: primero CLKPCE y, dentro
// de los cuatro ciclos siguientes, el nuevo valor del prescaler.
static void reloj_prescaler(uint8_t bits)
{
	uint8_t s = SREG;
	cli();
	CLKPR = (1 << CLKPCE);
	CLKPR = bits;
	SREG = s;
}

// Timer1 en CTC, sin prescaler, invirtiendo OC1A (D9) en cada
// comparacion: un cambio de D9 cada 16000 ciclos de reloj.
static void timer1_iniciar(void)
{
	TCCR1A = (1 << COM1A0);
	TCCR1B = 0;
	TCNT1 = 0;
	OCR1A = 15999;
	TCCR1B = (1 << WGM12) | (1 << CS10);
}

static void timer1_detener(void)
{
	TCCR1B = 0;
	TCCR1A = 0;
	PORTB &= ~(1 << TEST_PIN);
}

static void indicar(uint8_t n)
{
	for (uint8_t i = 0; i <= n; i++)
	{
		PORTB |= (1 << LED_PIN);
		_delay_ms(200);
		PORTB &= ~(1 << LED_PIN);
		_delay_ms(300);
	}
}

// Espera a que el boton quede suelto durante 50 ms seguidos.
static void esperar_liberacion(void)
{
	uint8_t estable = 0;

	while (estable < 50U)
	{
		if (PIND & (1 << BOTON_PIN))
		{
			estable++;
		}
		else
		{
			estable = 0;
		}

		_delay_ms(1);
	}
}

int main(void)
{
	cli();
	MCUSR = 0;
	wdt_disable();
	reloj_prescaler(0);

	DDRB |= (1 << LED_PIN) | (1 << TEST_PIN);
	PORTB &= ~((1 << LED_PIN) | (1 << TEST_PIN));

	DDRD &= ~(1 << BOTON_PIN);
	PORTD |= (1 << BOTON_PIN);

	EICRA = 0;   // INT0 por nivel bajo
	sei();

	uint8_t frecuencia = 0;

	for (;;)
	{
		// Indicacion siempre a 16 MHz, con D9 quieto.
		reloj_prescaler(0);
		timer1_detener();
		esperar_liberacion();
		indicar(frecuencia);

		timer1_iniciar();
		reloj_prescaler(clkpr_bits[frecuencia]);

		uint8_t pulsacion_valida = 0;

		while (!pulsacion_valida)
		{
			cli();
			boton = 0;
			EIMSK |= (1 << INT0);
			sei();

			// CPU activa, sin Sleep, a la frecuencia seleccionada.
			while (!boton)
			{
			}

			// La validacion se hace a 16 MHz para que el retardo
			// de 5 ms sea real; luego se restablece el prescaler.
			reloj_prescaler(0);
			_delay_ms(5);
			pulsacion_valida = ((PIND & (1 << BOTON_PIN)) == 0U);

			if (!pulsacion_valida)
			{
				reloj_prescaler(clkpr_bits[frecuencia]);
			}
		}

		frecuencia++;

		if (frecuencia >= N_FRECUENCIAS)
		{
			frecuencia = 0;
		}
	}
}


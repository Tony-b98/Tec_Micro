#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

#define LED_PIN   PB5   // D13
#define TEST_PIN  PB1   // D9
#define BOTON_PIN PD2   // D2 / INT0

#define N_MODOS      5U
#define REPETICIONES 5U

// Posicion 0 = ACTIVO (no se usa sleep).
static const uint8_t modos_sleep[N_MODOS] = {
	0,
	SLEEP_MODE_IDLE,
	SLEEP_MODE_ADC,
	SLEEP_MODE_PWR_DOWN,
	SLEEP_MODE_PWR_SAVE
};

volatile uint8_t despertar = 0;

// INT0 por nivel bajo: despierta al micro en todos los modos.
ISR(INT0_vect)
{
	PORTB |= (1 << TEST_PIN);   // primera accion observable
	EIMSK &= ~(1 << INT0);      // evita reentradas mientras se pulsa
	despertar = 1;
}

static void indicar_modo(uint8_t modo)
{
	for (uint8_t i = 0; i <= modo; i++)
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

	DDRB |= (1 << LED_PIN) | (1 << TEST_PIN);
	PORTB &= ~((1 << LED_PIN) | (1 << TEST_PIN));

	DDRD &= ~(1 << BOTON_PIN);
	PORTD |= (1 << BOTON_PIN);

	// Misma configuracion de perifericos que en la Actividad 1.
	PRR = 0;
	ADMUX = (1 << REFS0);
	ADCSRA =
	(1 << ADEN) |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);

	EICRA = 0;   // INT0 por nivel bajo

	uint8_t modo = 0;

	for (;;)
	{
		esperar_liberacion();
		indicar_modo(modo);

		for (uint8_t rep = 0; rep < REPETICIONES; rep++)
		{
			esperar_liberacion();

			// Pausa para rearmar el disparo del osciloscopio.
			_delay_ms(1000);

			PORTB &= ~(1 << TEST_PIN);

			cli();
			despertar = 0;
			EIMSK |= (1 << INT0);

			if (modo == 0U)
			{
				// ACTIVO: la CPU ejecuta instrucciones sin dormir.
				sei();

				while (!despertar)
				{
				}
			}
			else
			{
				set_sleep_mode(modos_sleep[modo]);

				while (!despertar)
				{
					sleep_enable();
					sei();
					sleep_cpu();
					sleep_disable();
					cli();
				}

				sei();
			}
		}

		modo++;

		if (modo >= N_MODOS)
		{
			modo = 0;
		}
	}
}


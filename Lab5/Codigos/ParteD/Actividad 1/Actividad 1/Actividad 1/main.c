// =====================================================
// LABORATORIO 5 - PROBLEMA D - ACTIVIDAD 1
// Comparacion de modos Sleep
// ATmega328P - ARDUINO UNO - 16 MHz
//
// Cada pulsacion del boton (D2 / INT0) pasa al modo siguiente:
//   1 parpadeo  : ACTIVO (CPU sin dormir)
//   2 parpadeos : Idle
//   3 parpadeos : ADC Noise Reduction
//   4 parpadeos : Power-Down
//   5 parpadeos : Power-Save
// Despues de los parpadeos el LED queda apagado y el micro
// permanece en ese modo hasta la siguiente pulsacion.
//
// D9 (PB1) es el pin de prueba: se pone en alto como primera
// accion al despertar. Sirve tambien para la Actividad 3.
// =====================================================

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

#define N_MODOS 5U

// Posicion 0 = ACTIVO (no se usa sleep).
static const uint8_t modos_sleep[N_MODOS] = {
	0,
	SLEEP_MODE_IDLE,
	SLEEP_MODE_ADC,
	SLEEP_MODE_PWR_DOWN,
	SLEEP_MODE_PWR_SAVE
};

volatile uint8_t despertar = 0;

// INT0 por nivel bajo: es la unica configuracion de INT0 que
// despierta al micro en todos los modos, incluido Power-Down.
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

	// Salidas: LED y pin de prueba, ambos en bajo.
	DDRB |= (1 << LED_PIN) | (1 << TEST_PIN);
	PORTB &= ~((1 << LED_PIN) | (1 << TEST_PIN));

	// Boton con pull-up interno.
	DDRD &= ~(1 << BOTON_PIN);
	PORTD |= (1 << BOTON_PIN);

	// Misma configuracion de perifericos en todos los modos:
	// PRR = 0 (todos los relojes habilitados) y ADC habilitado,
	// sin interrupcion, para que ADC Noise Reduction tenga sentido.
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

		modo++;

		if (modo >= N_MODOS)
		{
			modo = 0;
		}
	}
}

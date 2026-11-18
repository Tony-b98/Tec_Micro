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

#define N_INTERVALOS 4U

// Bits de prescaler del WDT para cada intervalo nominal.
static const uint8_t wdt_prescaler[N_INTERVALOS] = {
	0,                              // 16 ms
	(1 << WDP2),                    // 250 ms
	(1 << WDP2) | (1 << WDP1),      // 1 s
	(1 << WDP3) | (1 << WDP0)       // 8 s
};

volatile uint8_t boton = 0;

// Interrupcion del WDT: lo mas breve posible.
ISR(WDT_vect)
{
	PINB = (1 << TEST_PIN);   // escribir en PINB invierte el pin
}

// INT0 por nivel bajo: despierta al micro desde Power-Down.
ISR(INT0_vect)
{
	EIMSK &= ~(1 << INT0);    // evita reentradas mientras se pulsa
	boton = 1;
}

// WDT en modo interrupcion (WDIE = 1, WDE = 0) con el
// prescaler indicado. Secuencia temporizada del datasheet.
static void wdt_interrupcion(uint8_t prescaler)
{
	cli();
	wdt_reset();
	MCUSR &= ~(1 << WDRF);
	WDTCSR = (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDIE) | prescaler;
}

static void indicar_intervalo(uint8_t n)
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

	DDRB |= (1 << LED_PIN) | (1 << TEST_PIN);
	PORTB &= ~((1 << LED_PIN) | (1 << TEST_PIN));

	DDRD &= ~(1 << BOTON_PIN);
	PORTD |= (1 << BOTON_PIN);

	EICRA = 0;   // INT0 por nivel bajo

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	uint8_t intervalo = 0;

	for (;;)
	{
		// Con el WDT apagado D9 queda quieto durante los parpadeos.
		cli();
		wdt_disable();
		sei();

		esperar_liberacion();
		PORTB &= ~(1 << TEST_PIN);
		indicar_intervalo(intervalo);

		wdt_interrupcion(wdt_prescaler[intervalo]);

		uint8_t pulsacion_valida = 0;

		while (!pulsacion_valida)
		{
			cli();
			boton = 0;
			EIMSK |= (1 << INT0);

			// Duerme hasta que se pulse el boton. Las interrupciones
			// del WDT despiertan, invierten D9 y se vuelve a dormir.
			while (!boton)
			{
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
				cli();
			}

			sei();

			// Una pulsacion real mantiene el boton apretado;
			// un pico de ruido no.
			_delay_ms(5);
			pulsacion_valida = ((PIND & (1 << BOTON_PIN)) == 0U);
		}

		intervalo++;

		if (intervalo >= N_INTERVALOS)
		{
			intervalo = 0;
		}
	}
}


// =====================================================
// LABORATORIO 5 - PROBLEMA D - ACTIVIDAD 2
// Impacto de los perifericos mediante PRR
// ATmega328P - ARDUINO UNO - 16 MHz
//
// El micro permanece siempre en Idle. Cada pulsacion del
// boton (D2 / INT0) pasa a la configuracion siguiente:
//   1 parpadeo  : referencia, todos los perifericos habilitados
//   2 parpadeos : sin ADC
//   3 parpadeos : sin USART0
//   4 parpadeos : sin SPI
//   5 parpadeos : sin Timer1
//   6 parpadeos : sin Timer0
//   7 parpadeos : sin Timer2
//   8 parpadeos : sin TWI
//   9 parpadeos : todos deshabilitados
// Despues de los parpadeos el LED queda apagado y el micro
// permanece en esa configuracion hasta la siguiente pulsacion.
//
// El firmware no usa ningun temporizador para funcionar
// (los retardos son por software), por lo que deshabilitar
// Timer0, Timer1 o Timer2 no altera la prueba.
// =====================================================

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

#define LED_PIN   PB5   // D13
#define BOTON_PIN PD2   // D2 / INT0

// Mascara explicita con los siete perifericos (0xEF).
// No se escribe 0xFF: el bit 4 de PRR esta reservado.
#define PRR_TODOS ( \
(1 << PRADC)    | \
(1 << PRUSART0) | \
(1 << PRSPI)    | \
(1 << PRTIM1)   | \
(1 << PRTIM0)   | \
(1 << PRTIM2)   | \
(1 << PRTWI)      \
)

#define N_CONFIG 9U

static const uint8_t mascaras_prr[N_CONFIG] = {
	0,                  // 1: referencia
	(1 << PRADC),       // 2
	(1 << PRUSART0),    // 3
	(1 << PRSPI),       // 4
	(1 << PRTIM1),      // 5
	(1 << PRTIM0),      // 6
	(1 << PRTIM2),      // 7
	(1 << PRTWI),       // 8
	PRR_TODOS           // 9
};

volatile uint8_t despertar = 0;

ISR(INT0_vect)
{
	EIMSK &= ~(1 << INT0);   // nivel bajo: evita reentradas
	despertar = 1;
}

// Configuracion de referencia: todos los relojes habilitados y
// cada periferico encendido, sin interrupciones propias para
// que no despierten al micro desde Idle.
static void perifericos_habilitar(void)
{
	PRR = 0;

	// ADC habilitado, sin conversion en curso.
	ADMUX = (1 << REFS0);
	ADCSRA =
	(1 << ADEN) |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);

	// USART0 a 9600 bps, 8N1.
	UBRR0 = 103;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	// SPI como maestro (SS en salida para no pasar a esclavo).
	DDRB |= (1 << PB2);
	SPCR = (1 << SPE) | (1 << MSTR);

	// Timers contando, sin interrupciones.
	TIMSK1 = 0;
	TCCR1A = 0;
	TCCR1B = (1 << CS11) | (1 << CS10);

	TIMSK0 = 0;
	TCCR0A = 0;
	TCCR0B = (1 << CS01) | (1 << CS00);

	TIMSK2 = 0;
	TCCR2A = 0;
	TCCR2B = (1 << CS21);

	// TWI habilitado a 100 kHz.
	TWBR = 72;
	TWCR = (1 << TWEN);
}

// Apaga cada periferico indicado y luego detiene su reloj.
static void perifericos_deshabilitar(uint8_t mascara)
{
	if (mascara & (1 << PRADC))
	{
		ADCSRA = 0;
	}

	if (mascara & (1 << PRUSART0))
	{
		UCSR0B = 0;
	}

	if (mascara & (1 << PRSPI))
	{
		SPCR = 0;
	}

	if (mascara & (1 << PRTIM1))
	{
		TCCR1B = 0;
	}

	if (mascara & (1 << PRTIM0))
	{
		TCCR0B = 0;
	}

	if (mascara & (1 << PRTIM2))
	{
		TCCR2B = 0;
	}

	if (mascara & (1 << PRTWI))
	{
		TWCR = 0;
	}

	PRR |= mascara;
}

static void indicar_config(uint8_t config)
{
	for (uint8_t i = 0; i <= config; i++)
	{
		PORTB |= (1 << LED_PIN);
		_delay_ms(150);
		PORTB &= ~(1 << LED_PIN);
		_delay_ms(250);
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

	DDRB |= (1 << LED_PIN);
	PORTB &= ~(1 << LED_PIN);

	DDRD &= ~(1 << BOTON_PIN);
	PORTD |= (1 << BOTON_PIN);

	EICRA = 0;   // INT0 por nivel bajo

	set_sleep_mode(SLEEP_MODE_IDLE);

	uint8_t config = 0;

	for (;;)
	{
		// El SPI habilitado toma el control de D13 (SCK),
		// por eso se apaga antes de usar el LED.
		PRR = 0;
		SPCR = 0;

		esperar_liberacion();
		indicar_config(config);

		perifericos_habilitar();
		perifericos_deshabilitar(mascaras_prr[config]);

		cli();
		despertar = 0;
		EIMSK |= (1 << INT0);

		while (!despertar)
		{
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
			cli();
		}

		sei();

		config++;

		if (config >= N_CONFIG)
		{
			config = 0;
		}
	}
}

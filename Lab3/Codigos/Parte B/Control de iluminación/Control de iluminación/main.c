#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define F_CPU 16000000UL
#include <util/delay.h>

// L298N, puente H
#define MOTOR_IN1 PD4     // Arduino D4 -> IN1
#define MOTOR_IN2 PD5     // Arduino D5 -> IN2
#define MOTOR_ENA PD6     // Arduino D6 -> ENA

// PARAMETROS
// Banda alrededor del setpoint donde el motor se detiene
#define ZONA_MUERTA 40

// PWM minimo para vencer rozamiento
#define PWM_MIN 200

#define PWM_MAX 255

// Ajustar segun el recorrido real del potenciometro
#define POS_MIN 100
#define POS_MAX 900

// SETPOINT
volatile uint16_t setpoint = 500;

// UART RX
volatile char rx_buffer[5];
volatile uint8_t rx_index = 0;
volatile uint8_t comando_listo = 0;

// ADC
void ADC_init(void)
{
	// Referencia AVcc
	ADMUX = (1 << REFS0);

	// ADC habilitado
	// Prescaler 128
	ADCSRA =
	(1 << ADEN) |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);
}

uint16_t ADC_read(uint8_t channel)
{
	// Seleccionar ADC0, ADC1, etc.
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

	// Iniciar conversion
	ADCSRA |= (1 << ADSC);

	// Esperar final
	while (ADCSRA & (1 << ADSC));

	return ADC;
}

// UART
// 9600 baud - 8N1
void UART_init(void)
{
	UBRR0H = 0;
	UBRR0L = 103;

	// RX, TX e interrupcion de recepcion
	UCSR0B =
	(1 << RXEN0) |
	(1 << TXEN0) |
	(1 << RXCIE0);

	// 8 bits, sin paridad, 1 stop
	UCSR0C =
	(1 << UCSZ01) |
	(1 << UCSZ00);
}

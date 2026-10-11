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

// UART
void UART_send_char(char c)
{
	while (!(UCSR0A & (1 << UDRE0)));

	UDR0 = c;
}

void UART_send_string(const char *texto)
{
	while (*texto)
	{
		UART_send_char(*texto++);
	}
}

void UART_send_number(uint16_t numero)
{
	char buffer[6];
	uint8_t i = 0;

	if (numero == 0)
	{
		UART_send_char('0');
		return;
	}

	// Los digitos salen al reves, se envian desde el final.
	while (numero > 0)
	{
		buffer[i++] = (numero % 10) + '0';
		numero /= 10;
	}

	while (i > 0)
	{
		UART_send_char(buffer[--i]);
	}
}

// INTERRUPCION UART
ISR(USART_RX_vect)
{
	char c = UDR0;

	// Se ignora todo hasta procesar el comando anterior.
	if (comando_listo)
	{
		return;
	}

	if (c >= '0' && c <= '9')
	{
		// Maximo 4 digitos.
		if (rx_index < 4)
		{
			rx_buffer[rx_index++] = c;
		}
	}

	else if (c == '\r' || c == '\n')
	{
		if (rx_index > 0)
		{
			rx_buffer[rx_index] = '\0';
			comando_listo = 1;
		}
	}

	// Backspace
	else if (c == 8 || c == 127)
	{
		if (rx_index > 0)
		{
			rx_index--;
		}
	}
}

// PROCESAR SETPOINT
void procesar_setpoint(void)
{
	char buffer_local[5];

	uint8_t longitud;
	uint16_t nuevo = 0;

	if (!comando_listo)
	{
		return;
	}

	// Copia atomica del buffer de la ISR.
	cli();

	longitud = rx_index;

	for (uint8_t i = 0; i < longitud; i++)
	{
		buffer_local[i] = rx_buffer[i];
	}

	for (uint8_t i = 0; i < 5; i++)
	{
		rx_buffer[i] = 0;
	}

	rx_index = 0;
	comando_listo = 0;

	sei();

	for (uint8_t i = 0; i < longitud; i++)
	{
		nuevo = (nuevo * 10) +
		(buffer_local[i] - '0');
	}

	if (nuevo <= 1023)
	{
		setpoint = nuevo;

		UART_send_string("\r\n*** NUEVO SETPOINT: ");
		UART_send_number(setpoint);
		UART_send_string(" ***\r\n");
	}
	else
	{
		// Un valor invalido no se satura a 1023.
		UART_send_string("\r\nERROR: SETPOINT INVALIDO\r\n");
		UART_send_string("Ingrese un valor entre 0 y 1023\r\n");
	}
}// UART
void UART_send_char(char c)
{
	while (!(UCSR0A & (1 << UDRE0)));

	UDR0 = c;
}

void UART_send_string(const char *texto)
{
	while (*texto)
	{
		UART_send_char(*texto++);
	}
}

void UART_send_number(uint16_t numero)
{
	char buffer[6];
	uint8_t i = 0;

	if (numero == 0)
	{
		UART_send_char('0');
		return;
	}

	// Los digitos salen al reves, se envian desde el final.
	while (numero > 0)
	{
		buffer[i++] = (numero % 10) + '0';
		numero /= 10;
	}

	while (i > 0)
	{
		UART_send_char(buffer[--i]);
	}
}

// INTERRUPCION UART
ISR(USART_RX_vect)
{
	char c = UDR0;

	// Se ignora todo hasta procesar el comando anterior.
	if (comando_listo)
	{
		return;
	}

	if (c >= '0' && c <= '9')
	{
		// Maximo 4 digitos.
		if (rx_index < 4)
		{
			rx_buffer[rx_index++] = c;
		}
	}

	else if (c == '\r' || c == '\n')
	{
		if (rx_index > 0)
		{
			rx_buffer[rx_index] = '\0';
			comando_listo = 1;
		}
	}

	// Backspace
	else if (c == 8 || c == 127)
	{
		if (rx_index > 0)
		{
			rx_index--;
		}
	}
}

// PROCESAR SETPOINT
void procesar_setpoint(void)
{
	char buffer_local[5];

	uint8_t longitud;
	uint16_t nuevo = 0;

	if (!comando_listo)
	{
		return;
	}

	// Copia atomica del buffer de la ISR.
	cli();

	longitud = rx_index;

	for (uint8_t i = 0; i < longitud; i++)
	{
		buffer_local[i] = rx_buffer[i];
	}

	for (uint8_t i = 0; i < 5; i++)
	{
		rx_buffer[i] = 0;
	}

	rx_index = 0;
	comando_listo = 0;

	sei();

	for (uint8_t i = 0; i < longitud; i++)
	{
		nuevo = (nuevo * 10) +
		(buffer_local[i] - '0');
	}

	if (nuevo <= 1023)
	{
		setpoint = nuevo;

		UART_send_string("\r\n*** NUEVO SETPOINT: ");
		UART_send_number(setpoint);
		UART_send_string(" ***\r\n");
	}
	else
	{
		// Un valor invalido no se satura a 1023.
		UART_send_string("\r\nERROR: SETPOINT INVALIDO\r\n");
		UART_send_string("Ingrese un valor entre 0 y 1023\r\n");
	}
}

// PWM
// Timer0 - OC0A - D6
void PWM_init(void)
{
	DDRD |= (1 << MOTOR_ENA);

	// Fast PWM, OC0A no invertido
	TCCR0A =
	(1 << COM0A1) |
	(1 << WGM01) |
	(1 << WGM00);

	// Prescaler 64
	TCCR0B =
	(1 << CS01) |
	(1 << CS00);

	OCR0A = 0;
}

void PWM_set(uint8_t pwm)
{
	OCR0A = pwm;
}

// MOTOR
void MOTOR_init(void)
{
	DDRD |=
	(1 << MOTOR_IN1) |
	(1 << MOTOR_IN2);

	PORTD &= ~(1 << MOTOR_IN1);
	PORTD &= ~(1 << MOTOR_IN2);
}

void MOTOR_abrir(void)
{
	// Si físicamente gira al revés,
	// intercambiar estas dos líneas
	PORTD |=  (1 << MOTOR_IN1);
	PORTD &= ~(1 << MOTOR_IN2);
}

void MOTOR_cerrar(void)
{
	PORTD &= ~(1 << MOTOR_IN1);
	PORTD |=  (1 << MOTOR_IN2);
}

void MOTOR_stop(void)
{
	PWM_set(0);

	PORTD &= ~(1 << MOTOR_IN1);
	PORTD &= ~(1 << MOTOR_IN2);
}

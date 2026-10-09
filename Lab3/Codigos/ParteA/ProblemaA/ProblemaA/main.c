#define F_CPU 16000000UL

#define PCF8574 0x27
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "twi_lcd.h"

// CONSTANTES Y VARIABLES GLOBALES

// Punto medio de confort 
#define PM_DEFECTO       20
#define PM_MIN           10
#define T_MAX_SENSOR     50    // segun datasheet DHT11 

// Margen necesario para que el rango superior (PM + OFFSET_ALTO)
// nunca supere la temperatura maxima admitida por el sensor.

#define MARGEN_SENSOR    25
#define PM_MAX           (T_MAX_SENSOR - MARGEN_SENSOR)

// Umbrales en funcion del PM 
#define OFFSET_CALEF      5
#define OFFSET_BAJO       5
#define OFFSET_MEDIO     15
#define OFFSET_ALTO      25

// Niveles de PWM reportados del ventilador 
#define PWM_BAJA        102
#define PWM_MEDIA       178
#define PWM_ALTA        255

// Sensor DHT11: pin PC0 = A0 
#define DHT_PORT    PORTC
#define DHT_DDR     DDRC
#define DHT_PINR    PINC
#define DHT_BIT     PC0

// LEDs indicadores en PORTB 
#define LED_CALEF   PB1        // D9: calefactor
#define LED_BAJA    PB2        // D10: ventilador baja vel.
#define LED_MEDIA   PB3        // D11: ventilador media vel.
#define LED_ALTA    PB4        // D12: ventilador alta vel. 
// Variables de tiempo
volatile uint8_t segundos   = 0;
volatile uint8_t flag_medir = 0;

// Recepcion de comandos UART

volatile char     cmd_buf[16];
volatile uint8_t  cmd_idx     = 0;
volatile uint8_t  cmd_ready   = 0;
volatile uint8_t  cmd_invalid = 0;   // 1 = entrada rechazada
volatile uint8_t  cmd_p_cnt   = 0;   // conteo de 'P' de la linea

// Estado del sistema

uint8_t punto_medio = PM_DEFECTO;
uint8_t pwm_fan     = 0;
uint8_t calefactor  = 0;

// 1 mientras el usuario esta ingresando un nuevo PM
volatile uint8_t esperando_pm = 0;

// PROTOTIPOS DE FUNCIONES

// UART
static void uart_init(void);
static void uart_tx(char c);
static void uart_print(const char *s);

// DHT11
static uint8_t dht_esperar_mientras(uint8_t nivel);
static uint8_t dht11_leer(uint8_t *temp);

// LEDs
static void leds_init(void);
static void leds_off(void);
static void calef_set(uint8_t on);
static void fan_leds(uint8_t nivel);

// Timer1: base de 1 s
static void timer1_init(void);

// LCD
static void lcd_clear_safe(void);

// Logica de control
static uint8_t decidir_accion(uint8_t temperatura, uint8_t pm);
static const char *accion_txt(uint8_t accion);
static void aplicar_accion(uint8_t accion);

// LCD
static void lcd_mostrar(uint8_t temperatura);
static void lcd_error_sensor(uint8_t error);

// Menu UART
static void procesar_comando(void);
 
 
 // FUNCION PRINCIPAL 
 int main(void)
 {
	 uint8_t temperatura = 0;
	 uint8_t accion      = 1;
	 uint8_t error;

	 char linea_lcd[17];    // LCD: 16 caracteres + '\0'
	 char linea_uart[48];   // UART: mensajes largos (30 max)

	 // Inicializaciones
	 
	 uart_init();
	 leds_init();

	 twi_init();
	 twi_lcd_init();

	 // 0x06: incrementar cursor SIN desplazar display.
	 // 0x0C: display ON, cursor OFF, parpadeo OFF.
	 twi_lcd_cmd(0x06);
	 twi_lcd_cmd(0x0C);

	 // PC0 como entrada con pull-up para el DHT11.
	 DHT_DDR  &= ~(1 << DHT_BIT);
	 DHT_PORT |=  (1 << DHT_BIT);

	 timer1_init();
	 sei();

	 // Mensaje inicial por UART.
	 uart_print(
	 "\r\n"
	 "=== CONTROL DE TEMPERATURA ===\r\n"
	 "P = cambiar punto medio\r\n\r\n"
	 );

	 // LCD inicial (maximo 16 caracteres por linea).
	 lcd_clear_safe();
	 twi_lcd_cmd(0x80);
	 twi_lcd_msg("Control de Temp ");

	 snprintf(linea_lcd, sizeof(linea_lcd), "PM:%u C          ", punto_medio);
	 twi_lcd_cmd(0xC0);
	 twi_lcd_msg(linea_lcd);

	 // Esperar estabilizacion inicial del DHT11.
	 _delay_ms(2000);

	 // LOOP PRINCIPAL
	 
	 while (1)
	 {
		 
		 if (cmd_ready || cmd_invalid)
		 {
			 procesar_comando();
		 }

		 // Medicion cada 5 segundos (pausada durante cambio de PM).
		 if (flag_medir && !esperando_pm)
		 {
			 flag_medir = 0;

			 error = dht11_leer(&temperatura);

			 // LECTURA CORRECTA
			 if (error == 0)
			 {
				 accion = decidir_accion(temperatura, punto_medio);
				 aplicar_accion(accion);

				 // Caso maximo:"Temp=50 C | Accion=FAN MEDIA\r\n" = 30 chars.
				 snprintf(
				 linea_uart,
				 sizeof(linea_uart),
				 "Temp=%u C | Accion=%s\r\n",
				 temperatura,
				 accion_txt(accion)
				 );
				 uart_print(linea_uart);

				 lcd_mostrar(temperatura);
			 }

			 // ERROR DHT11: apagar salidas por seguridad
			 else
			 {
				 leds_off();
				 calefactor = 0;
				 pwm_fan    = 0;

				 snprintf(
				 linea_uart,
				 sizeof(linea_uart),
				 "MSG,ERROR DHT11 codigo=%u\r\n",
				 error
				 );
				 uart_print(linea_uart);

				 lcd_error_sensor(error);
			 }
		 }
	 }

	 return 0;   // inalcanzable
 }

//  4. IMPLEMENTACION DE FUNCIONES

// LCD: clear con espera obligatoria.
// El comando 0x01 (clear) del HD44780 tarda 1.52 ms.

static void lcd_clear_safe(void)
{
	twi_lcd_clear();
	_delay_ms(2);
}

//  UART
static void uart_init(void)
{
	UBRR0H = 0;
	UBRR0L = 103;                     // 9600 baudios @ 16 MHz

	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8N1
}

static void uart_tx(char c)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

static void uart_print(const char *s)
{
	while (*s)
	{
		uart_tx(*s++);
	}
}

// RECEPCION UART
// REGLAS DE ACEPTACION:
// Solo una 'P' habilita el menu
// CAMBIO DE PM:
// Solo digitos; al completarse 2 se procesa.
// Cualquier otro caracter CANCELA el ingreso por completo.

ISR(USART_RX_vect)
{
	char c = UDR0;

	// Enter (\r o \n): cerrar la linea
	if (c == '\r' || c == '\n')
	{
		if (!esperando_pm && cmd_idx > 0)
		{
			// Linea con caracteres que nunca tuvo una P inicial.
			cmd_invalid = 1;
		}

		if (!esperando_pm)
		{
			cmd_idx = 0;
		}

		return;
	}

	// Ignorar caracteres no imprimibles
	if (c < 0x20 || c > 0x7E)
	{
		return;
	}

	// Modo ingreso de PM: solo digitos
	if (esperando_pm)
	{
		if (c >= '0' && c <= '9')
		{
			if (cmd_idx < 2)
			{
				cmd_buf[cmd_idx++] = c;
			}

			if (cmd_idx == 2)
			{
				cmd_buf[2] = '\0';
				cmd_idx    = 0;
				cmd_ready  = 1;
			}
		}
		else
		{
			// Letra o simbolo durante el ingreso: se CANCELA el modo PM por completo.
			esperando_pm = 0;
			cmd_idx      = 0;
			cmd_p_cnt    = 0;
			cmd_invalid  = 1;   // main imprimira el error
		}

		return;
	}

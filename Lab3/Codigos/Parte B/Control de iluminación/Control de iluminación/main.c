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

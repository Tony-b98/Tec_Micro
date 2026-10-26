#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdint.h>

#define PCF8574  0x27
#include "twi_lcd.h"
#include "UART.c"

//Configuración de pines
#define WS_DDR          DDRD
#define WS_PORT         PORTD
#define WS_BIT          PD6                 // Datos WS2812B

#define BUZZ_DDR        DDRB
#define BUZZ_PORT       PORTB
#define BUZZ_BIT        PB0                 // Buzzer

#define BTN_HOLD        PD2                 // Botón HOLD
#define BTN_CAL         PD3                 // Botón CALIBRAR
#define BTN_DDR         DDRD
#define BTN_PORT        PORTD
#define BTN_PIN         PIND

// MPU6050 
#define MPU_ADDR        0x68                // AD0 = GND
#define MPU_ADDR_W      ((MPU_ADDR << 1) | 0)
#define MPU_ADDR_R      ((MPU_ADDR << 1) | 1)
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A                // filtro pasa bajos interno (DLPF)
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_ACCEL_XOUT  0x3B
#define MPU_PWR_MGMT    0x6B
#define MPU_WHO_AM_I    0x75

//Parámetros del sistema 
#define N_LEDS          64
#define MAX_ANGLE       30.0f               // ±30° -> bordes de la matriz
#define DEAD_ZONE       3.0f                // zona muerta ±3°
#define ALPHA           0.98f               // filtro complementario
#define EMA_A           0.70f               // suavizado adicional (media móvil exp.)
#define LOOP_MS         20                  // período del lazo (50 Hz)
#define EDGE_BEEP_MS    60                  // duración pitido de borde
#define BLINK_LOOPS     6                   // parpadeo en borde: 6*20 ms = 120 ms
#define DISP_LOOPS      10                  // refresco LCD/UART: 10*20 ms = 200 ms
#define GYRO_LSB_DPS    131.0f              // ±250 °/s -> 131 LSB por °/s
#define ACC_LSB_G       16384.0f            // ±2 g     -> 16384 LSB por g
#define RAD_TO_DEG      57.2958f
#define BAUD_UBRR       25                  // 38400 baud @ 16 MHz (error 0.2 %)

// Orientación de la matriz respecto al sensor (ajustar al montaje) 
#define INVERT_X        0                   // 1 = invertir sentido horizontal
#define INVERT_Y        0                   // 1 = invertir sentido vertical

/* Timer1 libre, prescaler 64 -> 4 us por cuenta (para medir dt real) */
#define T1_US_PER_TICK  4
#define LOOP_TICKS      ((LOOP_MS * 1000UL) / T1_US_PER_TICK)

// EEPROM: el compilador asigna las direcciones con EEMEM (sin casts) 
typedef struct {
	uint8_t magic;                          // MAGIC_OK = datos válidos
	float   roll, pitch;                    // offsets de calibración (°)
	float   gx, gy;                         // bias del giroscopio (LSB)
} EECal;
static EECal EEMEM ee;
#define MAGIC_OK        0xA7

// Instrucción NOP para el ensamblador en línea de la WS2812B 
#define NOP "nop\n\t"

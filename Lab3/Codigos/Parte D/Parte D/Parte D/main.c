#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

// CONFIGURACION
#ifndef CONFIG_H
#define CONFIG_H

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define UART_BAUD        9600UL

#define BUZ1_DDR   DDRB
#define BUZ1_PORT  PORTB
#define BUZ1_PIN   PB1
#define BUZ2_DDR   DDRB
#define BUZ2_PORT  PORTB
#define BUZ2_PIN   PB3

#define LCD_RS_DDR     DDRC
#define LCD_RS_PORT    PORTC
#define LCD_RS         PC4
#define LCD_E_DDR      DDRB
#define LCD_E_PORT     PORTB
#define LCD_E          PB2
#define LCD_DATA_DDR   DDRC
#define LCD_DATA_PORT  PORTC

#define TECLADO_PERIODO_MS   5   // ms
#define TECLADO_MUESTRAS     4   // lecturas iguales para aceptar
#define ARTICULACION_MS      18   // ms de silencio entre notas

#endif

// NOTAS
#ifndef NOTAS_H
#define NOTAS_H

#define SIL   0   // silencio
#define MANT  255   // mantener la nota anterior

#define NOTA_MIN  28
#define NOTA_MAX  96

#define N_E1     28
#define N_F1     29
#define N_FS1    30
#define N_G1     31
#define N_GS1    32
#define N_A1     33
#define N_AS1    34
#define N_B1     35
#define N_C2     36
#define N_CS2    37
#define N_D2     38
#define N_DS2    39
#define N_E2     40
#define N_F2     41
#define N_FS2    42
#define N_G2     43
#define N_GS2    44
#define N_A2     45
#define N_AS2    46
#define N_B2     47
#define N_C3     48
#define N_CS3    49
#define N_D3     50
#define N_DS3    51
#define N_E3     52
#define N_F3     53
#define N_FS3    54
#define N_G3     55
#define N_GS3    56
#define N_A3     57
#define N_AS3    58
#define N_B3     59
#define N_C4     60
#define N_CS4    61
#define N_D4     62
#define N_DS4    63
#define N_E4     64
#define N_F4     65
#define N_FS4    66
#define N_G4     67
#define N_GS4    68
#define N_A4     69
#define N_AS4    70
#define N_B4     71
#define N_C5     72
#define N_CS5    73
#define N_D5     74
#define N_DS5    75
#define N_E5     76
#define N_F5     77
#define N_FS5    78
#define N_G5     79
#define N_GS5    80
#define N_A5     81
#define N_AS5    82
#define N_B5     83
#define N_C6     84
#define N_CS6    85
#define N_D6     86
#define N_DS6    87
#define N_E6     88
#define N_F6     89
#define N_FS6    90
#define N_G6     91
#define N_GS6    92
#define N_A6     93
#define N_AS6    94
#define N_B6     95
#define N_C7     96

#endif

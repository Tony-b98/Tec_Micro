#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "UART.h"

//DEFINICIONES 
#define SIMULACION 0     //1 = Proteus, 0 = placa fisica
#define DEBUG_ADC  0     // 1 = imprime X/Y periodicamente por UART
#define INVERTIR_Y 1     // 1 si arriba/abajo quedan al reves
#define INVERTIR_X 0     // 1 si izquierda/derecha quedan al reves

#define LED_PIN PD7
#define SW_PIN PD2
#define RESET_PIN PD4
#define BUZZER_PIN PB3
#define JOY_X 0
#define JOY_Y 1
#define ZONA_PCT 30      // zona muerta = 30% del centro calibrado 
#define ZONA_MIN 10      // zona muerta minima en cuentas de ADC 
#define MOVE_MS 160
#define SHOW_ON_MS 500
#define SHOW_OFF_MS 200
#define LONGPRESS_MS 900
#define MAX_SEQ 16
#if SIMULACION
#define BRILLO 255       // Proteus dibuja el LED segun el brillo: 255 se ve bien 
#else
#define BRILLO 60        // placa real: 60 reduce mucho el consumo 
#endif
#define NCOLOR 4
#define POS(x,y) ((uint8_t)((y)*8+(x)))

typedef enum { ROJO=0, VERDE, AZUL, AMARILLO } color_t;
typedef enum { ST_LIBRE=0, ST_SHOW, ST_PLAY, ST_OK, ST_ERROR, ST_PAUSA } estado_t;
typedef enum { J_CENTRO=0, J_ARRIBA, J_ABAJO, J_IZQUIERDA, J_DERECHA } dir_t;

static const uint8_t colores[NCOLOR][3] = {
	{BRILLO,0,0}, {0,BRILLO,0}, {0,0,BRILLO}, {BRILLO,BRILLO,0}
};

static const uint16_t frecuencias[NCOLOR] = {392,523,262,659};
static uint8_t fb[64][3]; /* RGB */
static uint8_t seq_pos[MAX_SEQ], seq_color[MAX_SEQ];
static uint16_t EEMEM ee_record = 0;
static uint8_t EEMEM ee_nivel = 1;
static uint8_t EEMEM ee_estado = ST_LIBRE;
static volatile uint8_t reinicio_pendiente = 0;
static int16_t centro_x = 512, centro_y = 512, zona_muerta = 140;

// PROTOTIPOS DE FUNCIONES 

static void perifericos_init(void);
static void reiniciar_partida(uint8_t *x,uint8_t *y,uint8_t *color,
uint8_t *nivel,uint8_t *longitud,uint8_t *paso,
uint8_t *redibujar,estado_t *estado,uint16_t record);
static void adc_init(void);
static uint16_t adc_read(uint8_t canal);
static void joystick_calibrar(void);
static dir_t joystick_dir(void);
static uint8_t mover_cursor(uint8_t *x, uint8_t *y);
static uint8_t boton_evento(void);
static uint8_t color_aleatorio(uint8_t actual);
static void tono_init(void);
static void tono_on(uint16_t frecuencia);
static void tono_off(void);
static void ws_byte(uint8_t dato);
static void matriz_enviar(void);
static void matriz_limpiar(void);
static void matriz_pixel(uint8_t x,uint8_t y,uint8_t color,uint8_t tenue);
static void dibujar_cursor(uint8_t x,uint8_t y,uint8_t color);
static void dibujar_juego(uint8_t x,uint8_t y,uint8_t color,uint8_t paso);
static void generar_secuencia(uint8_t longitud);
static void reproducir_secuencia(uint8_t longitud);
static void animacion_acierto(void);
static void animacion_error(void);
static void animacion_victoria(void);
static void mostrar_numero(const char *etiqueta,uint16_t numero);
static void guardar_progreso(estado_t estado,uint8_t nivel,uint16_t record);


// IMPLEMENTACION DE FUNCIONES

ISR(PCINT2_vect)
{
    if(!(PIND & (1<<RESET_PIN))) reinicio_pendiente=1;
}
static void reiniciar_partida(uint8_t *x,uint8_t *y,uint8_t *color,
                             uint8_t *nivel,uint8_t *longitud,uint8_t *paso,
                             uint8_t *redibujar,estado_t *estado,uint16_t record)
{
    reinicio_pendiente=0;
    tono_off();
    matriz_limpiar();
    matriz_enviar();
    *x=3; *y=3; *color=ROJO;
    *nivel=1; *longitud=0; *paso=0;
    *redibujar=1; *estado=ST_LIBRE;
    guardar_progreso(*estado,*nivel,record);
    uart_print("Reinicio manual: nivel 1, record conservado\r\n");
}

static void perifericos_init(void)
{
    DDRD |= (1<<LED_PIN);
    PORTD &= ~(1<<LED_PIN);
    DDRD &= ~(1<<SW_PIN);
    PORTD |= (1<<SW_PIN);
    DDRD &= ~(1<<RESET_PIN);
    PORTD |= (1<<RESET_PIN);
    PCICR |= (1<<PCIE2);
    PCMSK2 |= (1<<PCINT20);
    sei();
    DDRC &= ~((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3));
    PORTC &= ~((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3));
}

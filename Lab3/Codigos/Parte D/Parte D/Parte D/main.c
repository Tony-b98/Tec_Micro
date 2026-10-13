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

// PROTOTIPOS
#ifndef TIEMPO_H
#define TIEMPO_H
void     iniciar_reloj(void);
uint32_t millis(void);
#endif
#ifndef SONIDO_H
#define SONIDO_H
#define VOZ_MELODIA  0   // buzzer 1, Timer1
#define VOZ_BAJO     1   // buzzer 2, Timer2
void        iniciar_buzzers(void);
void        tocar_nota(uint8_t voz, uint8_t nota);
void        apagar_buzzers(void);
uint16_t    frecuencia_nota(uint8_t nota);
void        nombre_nota(uint8_t nota, char *buf);
#endif

#ifndef TECLADO_H
#define TECLADO_H
void    iniciar_botones(void);
uint8_t leer_botones(void);
uint8_t botones_apretados(void);
#endif

#ifndef LCD_H
#define LCD_H
void lcd_init(void);
void lcd_limpiar(void);
void lcd_cursor(uint8_t fila, uint8_t col);
void lcd_letra(char c);
void lcd_escribir(const char *s);
void lcd_escribir_P(const char *s);
void lcd_fila(uint8_t fila, const char *s);
#endif

#ifndef UART_H
#define UART_H
void    uart_init(void);
void    uart_enviar(char c);
void    uart_print(const char *s);
void    uart_print_P(const char *s);
void    uart_print_num(uint16_t n);
uint8_t uart_leer_linea(char *buf, uint8_t max);
#endif

#ifndef CANCIONES_H
#define CANCIONES_H

typedef struct {
	uint8_t mel;   // nota MIDI, SIL o MANT
	uint8_t bajo;
	uint8_t dur;   // en unidades de ms_unidad
} paso_t;

typedef struct {
	const char   *nombre;
	const paso_t *pasos;
	uint16_t      cantidad;
	uint8_t       ms_unidad;   // ms por unidad de duracion
} cancion_t;

#define CANT_CANCIONES 3

extern const cancion_t canciones[CANT_CANCIONES] PROGMEM;
#endif

#ifndef REPRODUCTOR_H
#define REPRODUCTOR_H

void    empezar_cancion(uint8_t indice);
void    parar_cancion(void);
uint8_t actualizar_cancion(void);
uint8_t nota_actual(void);
uint8_t progreso_cancion(uint8_t escala);
void    nombre_cancion(uint8_t indice, char *buf, uint8_t max);

#endif

// TIEMPO
static volatile uint32_t ms_contador = 0;

ISR(TIMER0_COMPA_vect)
{
	ms_contador++;
}

void iniciar_reloj(void)
{
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A  = (uint8_t)(F_CPU / 64UL / 1000UL - 1);
	TIMSK0 = (1 << OCIE0A);
}
uint32_t millis(void)
{
	uint32_t t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		t = ms_contador;
	}
	return t;
}

// SONIDO
static const uint16_t tabla_frec[] PROGMEM = {
	41,   44,   46,   49,   52,   55,   58,   62,   65,   69,   73,   78,
	82,   87,   92,   98,  104,  110,  117,  123,  131,  139,  147,  156,
	165,  175,  185,  196,  208,  220,  233,  247,  262,  277,  294,  311,
	330,  349,  370,  392,  415,  440,  466,  494,  523,  554,  587,  622,
	659,  698,  740,  784,  831,  880,  932,  988, 1047, 1109, 1175, 1245,
	1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093,
};

static const char nombres[12][5] PROGMEM = {
	"Do", "Do#", "Re", "Re#", "Mi", "Fa", "Fa#", "Sol", "Sol#", "La", "La#", "Si"
};

// Prescalers de Timer2 en el orden de CS22..CS20 = 1..7.
static const uint16_t t2_div[] PROGMEM = { 1, 8, 32, 64, 128, 256, 1024 };

uint16_t frecuencia_nota(uint8_t nota)
{
	if (nota < NOTA_MIN || nota > NOTA_MAX) return 0;
	return pgm_read_word(&tabla_frec[nota - NOTA_MIN]);
}

void nombre_nota(uint8_t nota, char *buf)
{
	if (nota == SIL || nota == MANT) { buf[0] = '-'; buf[1] = '\0'; return; }
	strcpy_P(buf, nombres[nota % 12]);
	uint8_t i = 0;
	while (buf[i]) i++;
	buf[i++] = (char)('0' + (nota / 12 - 1));
	buf[i]   = '\0';
}

void iniciar_buzzers(void)
{
	BUZ1_DDR  |=  (1 << BUZ1_PIN);
	BUZ2_DDR  |=  (1 << BUZ2_PIN);
	BUZ1_PORT &= ~(1 << BUZ1_PIN);
	BUZ2_PORT &= ~(1 << BUZ2_PIN);
	TCCR1A = 0; TCCR1B = 0;
	TCCR2A = 0; TCCR2B = 0;
}

// Toggle por hardware en OC1A: la onda sale sin usar el CPU.
static void buzzer1(uint16_t f)
{
	if (f == 0) {
		TCCR1A = 0;
		TCCR1B = 0;
		BUZ1_PORT &= ~(1 << BUZ1_PIN);
		return;
	}
	uint16_t ocr = (uint16_t)((F_CPU / 16UL + f / 2) / f - 1);
	// Se reinicia TCNT1 para que no pase de largo el nuevo TOP.
	TCCR1B = 0;
	OCR1A  = ocr;
	TCNT1  = 0;
	TCCR1A = (1 << COM1A0);
	TCCR1B = (1 << WGM12) | (1 << CS11);
}
static void buzzer2(uint16_t f)
{
	if (f == 0) {
		TCCR2A = 0;
		TCCR2B = 0;
		BUZ2_PORT &= ~(1 << BUZ2_PIN);
		return;
	}
	// OCR2A es de 8 bits: se busca el prescaler mas chico que entre.
	for (uint8_t i = 0; i < 7; i++) {
		uint16_t n   = pgm_read_word(&t2_div[i]);
		uint32_t ocr = (F_CPU / 2UL / n + f / 2) / f;
		if (ocr >= 1 && ocr <= 256) {
			TCCR2B = 0;
			OCR2A  = (uint8_t)(ocr - 1);
			TCNT2  = 0;
			TCCR2A = (1 << COM2A0) | (1 << WGM21);
			TCCR2B = (uint8_t)(i + 1);
			return;
		}
	}
	buzzer2(0);
}

void tocar_nota(uint8_t voz, uint8_t nota)
{
	uint16_t f = (nota == SIL) ? 0 : frecuencia_nota(nota);
	if (voz == VOZ_MELODIA) buzzer1(f);
	else                    buzzer2(f);
}

void apagar_buzzers(void)
{
	buzzer1(0);
	buzzer2(0);
}

// ENTRADAS
#define MASK_D  0xFC   // PD2..PD7
#define MASK_B  ((1 << PB0) | (1 << PB4))   // D8 y D12

static uint8_t estable   = 0;
static uint8_t candidato = 0;
static uint8_t cuenta    = 0;

static uint8_t leer_pines(void)
{
	// Entradas activas en bajo, se invierten.
	uint8_t d = (uint8_t)(~PIND);
	uint8_t b = (uint8_t)(~PINB);
	uint8_t m = (uint8_t)((d & MASK_D) >> 2);
	if (b & (1 << PB0)) m |= (1 << 6);
	if (b & (1 << PB4)) m |= (1 << 7);
	return m;
}

void iniciar_botones(void)
{
	DDRD  &= (uint8_t)~MASK_D;
	PORTD |= MASK_D;
	DDRB  &= (uint8_t)~MASK_B;
	PORTB |= MASK_B;
}

uint8_t leer_botones(void)
{
	uint8_t r = leer_pines();
	// Antirrebote: el cambio se acepta tras TECLADO_MUESTRAS lecturas iguales.
	if (r != candidato) {
		candidato = r;
		cuenta = 0;
		return 0;
	}
	if (cuenta < TECLADO_MUESTRAS) cuenta++;
	if (cuenta == TECLADO_MUESTRAS && estable != candidato) {
		estable = candidato;
		return 1;
	}
	return 0;
}

uint8_t botones_apretados(void)
{
	return estable;
}

// LCD
static void lcd_pulso(void)
{
	LCD_E_PORT |=  (1 << LCD_E);
	_delay_us(1);
	LCD_E_PORT &= ~(1 << LCD_E);
	_delay_us(1);
}

static void lcd_nibble(uint8_t n)
{
	// Read-modify-write para no pisar RS, que comparte PORTC.
	LCD_DATA_PORT = (uint8_t)((LCD_DATA_PORT & 0xF0) | (n & 0x0F));
	lcd_pulso();
}

static void lcd_byte(uint8_t b, uint8_t es_dato)
{
	if (es_dato) LCD_RS_PORT |=  (1 << LCD_RS);
	else         LCD_RS_PORT &= ~(1 << LCD_RS);
	lcd_nibble(b >> 4);
	lcd_nibble(b);
	_delay_us(45);
}

static void lcd_comando(uint8_t c)
{
	lcd_byte(c, 0);
	// Clear y home tardan mas que el resto de las instrucciones.
	if (c <= 0x03) _delay_ms(2);
}

void lcd_init(void)
{
	LCD_RS_DDR   |=  (1 << LCD_RS);
	LCD_E_DDR    |=  (1 << LCD_E);
	LCD_DATA_DDR |=  0x0F;
	LCD_RS_PORT  &= ~(1 << LCD_RS);
	LCD_E_PORT   &= ~(1 << LCD_E);

	_delay_ms(50);

	// Secuencia de inicializacion por instrucciones del HD44780.
	lcd_nibble(0x03); _delay_ms(5);
	lcd_nibble(0x03); _delay_us(150);
	lcd_nibble(0x03); _delay_us(150);
	lcd_nibble(0x02); _delay_us(150);

	// 4 bits y 2 lineas, display ON sin cursor, incremento, clear.
	lcd_comando(0x28);
	lcd_comando(0x0C);
	lcd_comando(0x06);
	lcd_comando(0x01);
}

void lcd_limpiar(void)
{
	lcd_comando(0x01);
}

void lcd_cursor(uint8_t fila, uint8_t col)
{
	lcd_comando((uint8_t)(0x80 | ((fila ? 0x40 : 0x00) + col)));
}

void lcd_letra(char c)
{
	lcd_byte((uint8_t)c, 1);
}

void lcd_escribir(const char *s)
{
	while (*s) lcd_letra(*s++);
}

void lcd_escribir_P(const char *s)
{
	char c;
	while ((c = (char)pgm_read_byte(s++)) != '\0') lcd_letra(c);
}

void lcd_fila(uint8_t fila, const char *s)
{
	uint8_t i = 0;
	lcd_cursor(fila, 0);
	while (s[i] && i < 16) lcd_letra(s[i++]);
	while (i++ < 16) lcd_letra(' ');
}

// UART
#define TX_TAM 128   // potencia de 2 para usar mascara
#define RX_TAM 32

static volatile char    tx_buf[TX_TAM];
static volatile uint8_t tx_ini = 0, tx_fin = 0;
static volatile char    rx_buf[RX_TAM];
static volatile uint8_t rx_ini = 0, rx_fin = 0;

static char    linea[16];
static uint8_t linea_len = 0;

ISR(USART_RX_vect)
{
	char c = (char)UDR0;
	uint8_t sig = (uint8_t)((rx_fin + 1) & (RX_TAM - 1));
	// Si el buffer esta lleno el byte se descarta.
	if (sig != rx_ini) {
		rx_buf[rx_fin] = c;
		rx_fin = sig;
	}
}
ISR(USART_UDRE_vect)
{
	// Nada mas para enviar: se apaga la interrupcion.
	if (tx_ini == tx_fin) {
		UCSR0B &= ~(1 << UDRIE0);
		return;
	}
	UDR0 = (uint8_t)tx_buf[tx_ini];
	tx_ini = (uint8_t)((tx_ini + 1) & (TX_TAM - 1));
}

void uart_init(void)
{
	// U2X0 activo: doble velocidad, menor error a 9600.
	uint16_t ubrr = (uint16_t)(F_CPU / (8UL * UART_BAUD) - 1);
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)ubrr;
	UCSR0A = (1 << U2X0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

// Transmision por buffer para no frenar la cancion.
void uart_enviar(char c)
{
	uint8_t sig = (uint8_t)((tx_fin + 1) & (TX_TAM - 1));
	while (sig == tx_ini) { }
	tx_buf[tx_fin] = c;
	tx_fin = sig;
	UCSR0B |= (1 << UDRIE0);
}

void uart_print(const char *s)
{
	while (*s) uart_enviar(*s++);
}

void uart_print_P(const char *s)
{
	char c;
	while ((c = (char)pgm_read_byte(s++)) != '\0') uart_enviar(c);
}

void uart_print_num(uint16_t n)
{
	char b[6];
	uint8_t i = 0;
	do { b[i++] = (char)('0' + n % 10); n /= 10; } while (n);
	while (i) uart_enviar(b[--i]);
}

uint8_t uart_leer_linea(char *buf, uint8_t max)
{
	while (rx_ini != rx_fin) {
		char c = rx_buf[rx_ini];
		rx_ini = (uint8_t)((rx_ini + 1) & (RX_TAM - 1));

		if (c == '\r' || c == '\n') {
			// Ignora la linea vacia que deja el par \r\n.
			if (linea_len == 0) continue;
			uint8_t n = (linea_len < max - 1) ? linea_len : (uint8_t)(max - 1);
			for (uint8_t i = 0; i < n; i++) buf[i] = linea[i];
			buf[n] = '\0';
			linea_len = 0;
			return 1;
		}
		if (linea_len < sizeof(linea)) linea[linea_len++] = c;
	}
	return 0;
}

// CANCIONES
static const paso_t pasos_para_elisa[] PROGMEM = {
	{N_E5,   SIL,      1}, {N_DS5,  SIL,      1}, {N_E5,   SIL,      1}, {N_DS5,  SIL,      1},
	{N_E5,   SIL,      1}, {N_B4,   SIL,      1}, {N_D5,   SIL,      1}, {N_C5,   SIL,      1},
	{N_A4,   N_A3,     1}, {MANT,   N_E4,     1}, {SIL,    N_A4,     1}, {N_C4,   SIL,      1},
	{N_E4,   SIL,      1}, {N_A4,   SIL,      1}, {N_B4,   N_E3,     1}, {MANT,   N_E4,     1},
	{SIL,    N_GS4,    1}, {N_E4,   SIL,      1}, {N_GS4,  SIL,      1}, {N_B4,   SIL,      1},
	{N_C5,   N_A3,     1}, {MANT,   N_E4,     1}, {SIL,    N_A4,     1}, {N_E4,   SIL,      1},
	{N_E5,   SIL,      1}, {N_DS5,  SIL,      1}, {N_E5,   SIL,      1}, {N_DS5,  SIL,      1},
	{N_E5,   SIL,      1}, {N_B4,   SIL,      1}, {N_D5,   SIL,      1}, {N_C5,   SIL,      1},
	{N_A4,   N_A3,     1}, {MANT,   N_E4,     1}, {SIL,    N_A4,     1}, {N_C4,   SIL,      1},
	{N_E4,   SIL,      1}, {N_A4,   SIL,      1}, {N_B4,   N_E3,     1}, {MANT,   N_E4,     1},
	{SIL,    N_GS4,    1}, {N_E4,   SIL,      1}, {N_C5,   SIL,      1}, {N_B4,   SIL,      1},
	{N_A4,   N_A3,     1}, {MANT,   N_E4,     1}, {MANT,   N_A4,     1}, {MANT,   SIL,      1},
	{N_A4,   N_A3,     1}, {MANT,   N_E4,     1}, {SIL,    N_A4,     1}, {N_B4,   SIL,      1},
	{N_C5,   SIL,      1}, {N_D5,   SIL,      1}, {N_E5,   N_C4,     1}, {MANT,   N_G4,     1},
	{MANT,   N_C5,     1}, {N_G4,   SIL,      1}, {N_F5,   SIL,      1}, {N_E5,   SIL,      1},
	{N_D5,   N_G3,     1}, {MANT,   N_G4,     1}, {MANT,   N_B4,     1}, {N_F4,   SIL,      1},
	{N_E5,   SIL,      1}, {N_D5,   SIL,      1}, {N_C5,   N_A3,     1}, {MANT,   N_E4,     1},
	{MANT,   N_A4,     1}, {N_E4,   SIL,      1}, {N_D5,   SIL,      1}, {N_C5,   SIL,      1},
	{N_B4,   N_E3,     1}, {MANT,   N_E4,     1}, {SIL,    N_E5,     1}, {N_E4,   SIL,      1},
	{N_E5,   SIL,      1}, {SIL,    N_E5,     1}, {SIL,    N_E6,     1}, {N_E5,   SIL,      1},
	{N_E6,   SIL,      1}, {SIL,    N_DS6,    1}, {SIL,    N_E6,     1}, {N_DS5,  SIL,      1},
	{N_E5,   SIL,      1}, {MANT,   N_DS6,    1}, {SIL,    N_E6,     1}, {N_DS5,  SIL,      1},
	{N_E5,   SIL,      1}, {N_DS5,  SIL,      1}, {N_E5,   SIL,      1}, {N_DS5,  SIL,      1},
	{N_E5,   SIL,      1}, {N_B4,   SIL,      1}, {N_D5,   SIL,      1}, {N_C5,   SIL,      1},
	{N_A4,   N_A3,     1}, {MANT,   N_E4,     1}, {SIL,    N_A4,     1}, {N_C4,   SIL,      1},
	{N_E4,   SIL,      1}, {N_A4,   SIL,      1}, {N_B4,   N_E3,     1}, {MANT,   N_E4,     1},
	{SIL,    N_GS4,    1}, {N_E4,   SIL,      1}, {N_GS4,  SIL,      1}, {N_B4,   SIL,      1},
	{N_C5,   N_A3,     1}, {MANT,   N_E4,     1}, {SIL,    N_A4,     1}, {N_E4,   SIL,      1},
	{N_E5,   SIL,      1}, {N_DS5,  SIL,      1}, {N_E5,   SIL,      1}, {N_DS5,  SIL,      1},
	{N_E5,   SIL,      1}, {N_B4,   SIL,      1}, {N_D5,   SIL,      1}, {N_C5,   SIL,      1},
	{N_A4,   N_A3,     1}, {MANT,   N_E4,     1}, {MANT,   N_A4,     7},
};

static const paso_t pasos_marcha_nupcial[] PROGMEM = {
	{N_C4,   SIL,      4}, {N_C4,   SIL,      4}, {N_C4,   SIL,      4}, {N_C4,   SIL,     24},
	{SIL,    SIL,     12}, {N_C4,   SIL,      4}, {N_C4,   SIL,      4}, {N_C4,   SIL,      4},
	{N_C4,   SIL,     24}, {SIL,    SIL,     12}, {N_C4,   SIL,      4}, {N_C4,   SIL,      4},
	{N_C4,   SIL,      4}, {N_E4,   SIL,     12}, {N_E4,   N_C3,     4}, {N_E4,   N_C3,     4},
	{N_E4,   N_C3,     4}, {N_E4,   N_C3,    12}, {N_E4,   N_C3,     4}, {N_E4,   N_C3,     4},
	{N_E4,   N_C3,     4}, {N_G4,   N_C3,    12}, {N_G4,   N_C3,     4}, {N_G4,   N_C3,     4},
	{N_G4,   N_C3,     4}, {N_G4,   N_C3,    12}, {SIL,    N_C3,     4}, {SIL,    N_C3,     4},
	{SIL,    N_C3,     4}, {N_C6,   N_A3,    24}, {N_B5,   N_B3,    18}, {N_FS5,  N_B3,     6},
	{N_A5,   N_E4,    12}, {N_G5,   MANT,    12}, {N_F5,   N_F4,     6}, {SIL,    SIL,      6},
	{N_D5,   N_F4,     6}, {SIL,    SIL,      6}, {N_C5,   N_G3,    23}, {SIL,    MANT,     1},
	{N_D5,   N_G4,    12}, {N_G4,   MANT,     9}, {N_D5,   MANT,     3}, {N_E5,   N_C4,    12},
	{SIL,    SIL,     18}, {N_C5,   SIL,      3}, {SIL,    SIL,      3}, {N_E5,   SIL,      3},
	{SIL,    SIL,      3}, {N_G5,   SIL,      3}, {SIL,    SIL,      3}, {N_C6,   N_A3,    24},
	{N_B5,   N_B3,    18}, {N_FS5,  N_B3,     6}, {N_A5,   N_E4,    12}, {N_G5,   MANT,    12},
	{N_F5,   N_F4,     6}, {SIL,    SIL,      6}, {N_D5,   N_F4,     6}, {SIL,    SIL,      6},
	{N_C5,   N_G3,    23}, {SIL,    MANT,     1}, {N_E5,   N_G4,    12}, {N_D5,   N_G3,     9},
	{N_E5,   MANT,     3}, {N_D5,   N_G3,    24}, {N_C5,   N_C4,    12}, {MANT,   MANT,    36},
};

static const paso_t pasos_arroz_con_leche[] PROGMEM = {
	{N_C4,  SIL,  1},
	{N_F4,  N_F3, 2}, {N_A4,  MANT, 1}, {N_F4,  N_C3, 1}, {N_F4,  MANT, 1}, {N_C4,  MANT, 1},
	{N_F4,  N_F3, 1}, {N_F4,  MANT, 1}, {N_A4,  MANT, 1}, {N_G4,  N_C3, 2}, {N_G4,  MANT, 1},
	{N_AS4, N_C3, 1}, {N_AS4, MANT, 1}, {N_AS4, MANT, 1}, {N_A4,  N_F3, 1}, {N_A4,  MANT, 1}, {N_A4, MANT, 1},
	{N_G4,  N_C3, 1}, {N_A4,  MANT, 1}, {N_G4,  MANT, 1}, {N_F4,  N_F3, 2}, {N_C4,  MANT, 1},
	{N_F4,  N_F3, 1}, {N_F4,  MANT, 1}, {N_A4,  MANT, 1}, {N_F4,  N_C3, 2}, {N_C4,  MANT, 1},
	{N_F4,  N_F3, 1}, {N_F4,  MANT, 1}, {N_A4,  MANT, 1}, {N_G4,  N_C3, 2}, {N_G4,  MANT, 1},
	{N_AS4, N_C3, 1}, {N_AS4, MANT, 1}, {N_AS4, MANT, 1}, {N_A4,  N_F3, 1}, {N_A4,  MANT, 1}, {N_A4, MANT, 1},
	{N_G4,  N_C3, 2}, {N_G4,  MANT, 1}, {N_F4,  N_F3, 2}, {N_A4,  MANT, 1},
	{N_C5,  N_C3, 2}, {N_C4,  MANT, 1}, {N_F4,  N_F3, 2}, {N_A4,  MANT, 1},
	{N_C5,  N_C3, 1}, {N_C5,  MANT, 1}, {N_C4,  MANT, 1}, {N_F4,  N_F3, 2}, {N_C4,  MANT, 1},
	{N_F4,  N_F3, 1}, {N_F4,  MANT, 1}, {N_A4,  MANT, 1}, {N_F4,  N_C3, 1}, {N_F4,  MANT, 1}, {N_F4, MANT, 1},
	{N_F4,  N_F3, 1}, {N_E4,  MANT, 1}, {N_F4,  MANT, 1}, {N_G4,  N_C3, 2}, {N_G4,  MANT, 1},
	{N_AS4, N_C3, 1}, {N_AS4, MANT, 1}, {N_AS4, MANT, 1}, {N_A4,  N_F3, 1}, {N_A4,  MANT, 1}, {N_A4, MANT, 1},
	{N_G4,  N_C3, 1}, {N_G4,  MANT, 1}, {N_G4,  MANT, 1}, {N_F4,  N_F3, 6},
};

static const char nombre_c1[] PROGMEM = "Para Elisa";
static const char nombre_c2[] PROGMEM = "Arroz con leche";
static const char nombre_c3[] PROGMEM = "Marcha nupcial";

#define CANT(x) (sizeof(x) / sizeof((x)[0]))

// Ultimo campo: ms por unidad de duracion (tempo).
const cancion_t canciones[CANT_CANCIONES] PROGMEM = {
	{ nombre_c1, pasos_para_elisa,      CANT(pasos_para_elisa),      160 },
	{ nombre_c2, pasos_arroz_con_leche, CANT(pasos_arroz_con_leche), 210 },
	{ nombre_c3, pasos_marcha_nupcial,  CANT(pasos_marcha_nupcial),   45 },
};

// REPRODUCTOR
static cancion_t c;
static uint16_t  indice_paso;
static uint32_t  t_inicio;
static uint16_t  dur_ms;
static uint8_t   cortar_mel, cortar_bajo, cortado;
static uint8_t   nota_mel;
static uint8_t   activo = 0;

static void leer_paso(uint16_t i, paso_t *p)
{
	memcpy_P(p, &c.pasos[i], sizeof(paso_t));
}

static void cargar_paso(uint16_t i)
{
	paso_t p, sig;
	leer_paso(i, &p);

	if (p.mel != MANT)  { tocar_nota(VOZ_MELODIA, p.mel); nota_mel = p.mel; }
	if (p.bajo != MANT)   tocar_nota(VOZ_BAJO, p.bajo);

	dur_ms = (uint16_t)p.dur * c.ms_unidad;

	// Si el paso siguiente ataca una nota nueva, esa voz se corta antes.
	if (i + 1 < c.cantidad) {
		leer_paso(i + 1, &sig);
		cortar_mel  = (sig.mel  != MANT);
		cortar_bajo = (sig.bajo != MANT);
		} else {
		cortar_mel = cortar_bajo = 1;
	}
	cortado = 0;
}

void empezar_cancion(uint8_t indice)
{
	memcpy_P(&c, &canciones[indice], sizeof(cancion_t));
	indice_paso = 0;
	activo = 1;
	apagar_buzzers();
	t_inicio = millis();
	cargar_paso(0);
}

void parar_cancion(void)
{
	activo = 0;
	nota_mel = SIL;
	apagar_buzzers();
}

uint8_t actualizar_cancion(void)
{
	if (!activo) return 0;

	// No bloqueante: se compara contra millis() en cada vuelta.
	uint32_t t = millis() - t_inicio;

	if (!cortado && dur_ms > 2 * ARTICULACION_MS && t >= dur_ms - ARTICULACION_MS) {
		if (cortar_mel)  tocar_nota(VOZ_MELODIA, SIL);
		if (cortar_bajo) tocar_nota(VOZ_BAJO, SIL);
		cortado = 1;
	}

	if (t >= dur_ms) {
		// Suma exacta para no acumular error de tiempo.
		t_inicio += dur_ms;
		indice_paso++;
		if (indice_paso >= c.cantidad) {
			parar_cancion();
			return 0;
		}
		cargar_paso(indice_paso);
	}
	return 1;
}

uint8_t nota_actual(void)
{
	return nota_mel;
}

uint8_t progreso_cancion(uint8_t escala)
{
	if (!activo || c.cantidad == 0) return 0;
	return (uint8_t)(((uint32_t)indice_paso * escala) / c.cantidad);
}

void nombre_cancion(uint8_t indice, char *buf, uint8_t max)
{
	const char *p = (const char *)pgm_read_ptr(&canciones[indice].nombre);
	strncpy_P(buf, p, max - 1);
	buf[max - 1] = '\0';
}

// MAQUINA DE ESTADOS
typedef enum { EST_PIANO, EST_CANCION } estado_t;

static estado_t estado;
static uint8_t  cancion_actual;
static uint8_t  teclas_previas;

static const uint8_t notas_piano[8] PROGMEM = {
	N_C4, N_D4, N_E4, N_F4, N_G4, N_A4, N_B4, N_C5
};

// PANTALLA
static void pantalla_piano(void)
{
	lcd_fila(0, "  MODO  PIANO");
	lcd_fila(1, "Toque una tecla");
}

static void pantalla_nota(uint8_t mel, uint8_t bajo, uint8_t acorde)
{
	char l[17], n[6];
	nombre_nota(mel, n);
	strcpy(l, n);
	if (acorde) {
		strcat(l, " + ");
		nombre_nota(bajo, n);
		strcat(l, n);
		} else {
		uint16_t f = frecuencia_nota(mel);
		uint8_t i = (uint8_t)strlen(l);
		while (i < 6) l[i++] = ' ';
		char d[6]; uint8_t k = 0;
		do { d[k++] = (char)('0' + f % 10); f /= 10; } while (f);
		while (k) l[i++] = d[--k];
		l[i] = '\0';
		strcat(l, " Hz");
	}
	lcd_fila(1, l);
}

static void pantalla_cancion(uint8_t nota, uint8_t prog)
{
	char l[17], n[6];
	l[0] = 'C'; l[1] = (char)('1' + cancion_actual); l[2] = ' ';
	nombre_nota(nota, n);
	uint8_t i = 3, k = 0;
	while (n[k] && i < 8) l[i++] = n[k++];
	while (i < 8) l[i++] = ' ';
	// 0xFF es el bloque lleno del LCD.
	for (k = 0; k < 8; k++) l[i++] = (k < prog) ? (char)0xFF : '.';
	l[i] = '\0';
	lcd_fila(1, l);
}

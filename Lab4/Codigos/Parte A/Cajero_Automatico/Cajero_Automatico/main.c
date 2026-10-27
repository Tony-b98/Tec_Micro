#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "UART.h"
#include "SPI.h"
#include "RC522.h"

/* twi_lcd.h hace #include "twi.c", y twi.c también se compila aparte
   en el proyecto, lo que duplica las funciones twi_*. Renombramos la
   copia que entra en este archivo para que no choque con la otra. */
#define twi_init           lcd_twi_init
#define twi_start          lcd_twi_start
#define twi_repeated_start lcd_twi_repeated_start
#define twi_write_cmd      lcd_twi_write_cmd
#define twi_write_dwr      lcd_twi_write_dwr
#define twi_stop           lcd_twi_stop
#define twi_read_ack       lcd_twi_read_ack
#define twi_read_nack      lcd_twi_read_nack

#define PCF8574 0x27  
#include "twi_lcd.h"

#define BAUD_PRESCALE 103
#define LED_VERDE PC0
#define LED_ROJO PC1
#define BUZZER PC2
#define ROW_MASK 0xF0
#define COL_C1 PD2
#define COL_C2 PD3
#define COL_C3 PC3 
#define COL_C4 PB0

typedef struct { uint8_t uid[4]; uint8_t pin[4]; uint32_t saldo; } cuenta_t;
#define N_CUENTAS 3

/* Cambiar EE_MAGIC cada vez que se modifique la tabla por defecto
   (UIDs, PINs o saldos) para forzar que se copie de nuevo a la EEPROM. */
#define EE_MAGIC 0xA7

/* Tabla por defecto en flash: se copia a EEPROM la primera vez */
static const cuenta_t cuentas_def[N_CUENTAS] = {
	{{0xD3,0x9A,0x88,0x29},{'1','2','3','4'},5000},
	{{0xCA,0x97,0xF6,0x34},{'5','5','5','5'},3000},
	{{0xE3,0xF8,0x13,0x28},{'9','9','9','9'},1000}
};

static cuenta_t EEMEM ee_cuentas[N_CUENTAS];
static uint8_t  EEMEM ee_magic;
static cuenta_t cuentas[N_CUENTAS];

static uint8_t cuenta_actual=0xFF;

typedef enum { ST_ESPERA_TARJETA,ST_PIN,ST_MENU,ST_MONTO_RETIRO,
ST_MONTO_DEPOSITO,ST_MOSTRAR_SALDO,ST_BLOQUEO } estado_t;

static estado_t estado=ST_ESPERA_TARJETA;
static char pin_buf[5];

static uint8_t pin_idx=0,intentos=0;

static uint32_t monto=0;

static const char teclas[4][4]={{'1','2','3','A'},{'4','5','6','B'},
{'7','8','9','C'},{'*','0','#','D'}};

/* ---------- Prototipos de funciones ---------- */
static void keypad_init(void);
static uint8_t col_pressed(uint8_t c);
static char keypad_scan(void);
static void indicadores_init(void);
static void ok(void);
static void error_led(void);
static void buzz(void);
static void cargar_cuentas(void);
static void guardar_saldo(uint8_t idx);
static uint8_t detectar_tarjeta(uint8_t *uid);
static void lcd_num(uint32_t numero);
static void lcd_goto(uint8_t col,uint8_t fila);
static void lcd_show(const char *l1,const char *l2);
static uint32_t leer_monto(const char *titulo);


// ---------- Programa principal ---------- 

int main(void)
{
	uart_init(BAUD_PRESCALE);
	uart_print("\r\n=== CAJERO ATM INICIANDO ===\r\n");

	spi_init();
	mfrc522_resetPinInit();
	mfrc522_init();

	twi_init();
	keypad_init();
	indicadores_init();
	cargar_cuentas();
	twi_lcd_init();
	twi_lcd_cmd(0x0C);   /* display on, sin cursor parpadeante */

	lcd_show("CAJERO ATM", "Acerque tarjeta");

	/* mfrc522_standard() borra 16 bytes cuando no hay tarjeta,
	   así que el buffer debe tener al menos 16 */
	uint8_t uid[16];
	estado_t estado_previo = ST_BLOQUEO;   /* distinto al inicial */

	for (;;)
	{
		/* entrando = 1 solo en la primera vuelta de cada estado */
		uint8_t entrando = (estado != estado_previo);
		estado_previo = estado;

		switch (estado)
		{
			case ST_ESPERA_TARJETA:
			if (detectar_tarjeta(uid))
			{
				uart_print("UID detectado: ");
				uart_print_hex_array(uid, 4);

				cuenta_actual = 0xFF;

				for (uint8_t i = 0; i < N_CUENTAS; i++)
				{
					if (memcmp(cuentas[i].uid, uid, 4) == 0)
					{
						cuenta_actual = i;
						break;
					}
				}

				if (cuenta_actual != 0xFF)
				{
					ok();
					lcd_show("Bienvenido", 0);
					lcd_num(cuenta_actual + 1);

					pin_idx = 0;
					intentos = 0;
					estado = ST_PIN;

					_delay_ms(800);
					lcd_show("Ingrese PIN", "PIN: ");
				}
				else
				{
					error_led();
					lcd_show("Tarjeta no val", 0);

					_delay_ms(1500);
					lcd_show("Acerque tarjeta", 0);
				}
			}

			_delay_ms(300);
			break;

			case ST_PIN:
			{
				char k = keypad_scan();

				if (k)
				{
					/* Depuración: muestra por UART la tecla que leyó el teclado */
					uart_print("Tecla: ");
					uart_send(k);
					uart_print("\r\n");
				}

				if (k >= '0' && k <= '9' && pin_idx < 4)
				{
					pin_buf[pin_idx++] = k;
					twi_lcd_dwr('*');
				}
				else if (k == '*' && pin_idx > 0)   /* borrar último dígito */
				{
					pin_idx--;
					lcd_goto(5 + pin_idx, 1);
					twi_lcd_dwr(' ');
					lcd_goto(5 + pin_idx, 1);
				}

				if (pin_idx == 4)
				{
					pin_buf[4] = 0;

					if (memcmp(pin_buf, cuentas[cuenta_actual].pin, 4) == 0)
					{
						ok();
						intentos = 0;

						lcd_show("Acceso OK", 0);
						uart_print("PIN correcto\r\n");

						_delay_ms(1000);
						estado = ST_MENU;
					}
					else
					{
						error_led();
						intentos++;

						lcd_show("PIN erroneo", 0);

						uart_print("PIN erroneo, intento N: ");
						uart_print_hex(intentos);
						uart_print("\r\n");

						_delay_ms(1000);

						if (intentos >= 3)
						{
							estado = ST_BLOQUEO;
						}
						else
						{
							lcd_show("Ingrese PIN", "PIN: ");
						}

						pin_idx = 0;
					}
				}

				break;
			}

			case ST_BLOQUEO:
			lcd_show("BLOQUEADO", 0);
			uart_print("SISTEMA BLOQUEADO\r\n");

			buzz();
			_delay_ms(3000);

			intentos = 0;
			pin_idx = 0;
			cuenta_actual = 0xFF;

			estado = ST_ESPERA_TARJETA;
			lcd_show("Acerque tarjeta", 0);
			break;

			case ST_MENU:
			{
				/* Dibujar el menú una sola vez al entrar (antes se
				   redibujaba en cada vuelta y casi no leía el teclado) */
				if (entrando)
				{
					lcd_show("A:Ret B:Dep", "C:Saldo D:Salir");
				}

				char k = keypad_scan();

				if (k == 'A')
				{
					estado = ST_MONTO_RETIRO;
				}
				else if (k == 'B')
				{
					estado = ST_MONTO_DEPOSITO;
				}
				else if (k == 'C')
				{
					estado = ST_MOSTRAR_SALDO;
				}
				else if (k == 'D')
				{
					cuenta_actual = 0xFF;

					lcd_show("Hasta luego", 0);
					_delay_ms(1000);

					estado = ST_ESPERA_TARJETA;
					lcd_show("Acerque tarjeta", 0);
				}

				break;
			}

			case ST_MONTO_RETIRO:
			monto = leer_monto("Retirar monto:");

			uart_print("Retiro solicitado: ");
			uart_print_hex((uint8_t)(monto >> 8));
			uart_print_hex((uint8_t)monto);
			uart_print("\r\n");

			if (monto <= cuentas[cuenta_actual].saldo)
			{
				cuentas[cuenta_actual].saldo -= monto;
				guardar_saldo(cuenta_actual);

				ok();
				lcd_show("Retiro OK", 0);
			}
			else
			{
				error_led();
				lcd_show("Saldo insuf.", 0);
			}

			_delay_ms(1500);
			estado = ST_MENU;
			break;

			case ST_MONTO_DEPOSITO:
			monto = leer_monto("Depositar monto:");

			if (monto <= UINT32_MAX - cuentas[cuenta_actual].saldo)
			{
				cuentas[cuenta_actual].saldo += monto;
				guardar_saldo(cuenta_actual);

				ok();
				lcd_show("Deposito OK", 0);
			}
			else
			{
				error_led();
				lcd_show("Monto invalido", 0);
			}

			_delay_ms(1500);
			estado = ST_MENU;
			break;

			case ST_MOSTRAR_SALDO:
			twi_lcd_clear();
			_delay_ms(3);

			lcd_goto(0, 0);
			_delay_ms(3);

			twi_lcd_msg("Saldo: ");
			lcd_num(cuentas[cuenta_actual].saldo);

			while (!keypad_scan());

			estado = ST_MENU;
			break;
		}
	}
}

// Implementacion de funciones 

static void keypad_init(void) {
	DDRD |= ROW_MASK; PORTD |= ROW_MASK;
	DDRD &= ~((1<<COL_C1)|(1<<COL_C2));
	PORTD |= (1<<COL_C1)|(1<<COL_C2);
	DDRC &= ~(1<<COL_C3); PORTC |= (1<<COL_C3);
	DDRB &= ~(1<<COL_C4); PORTB |= (1<<COL_C4);
}

static uint8_t col_pressed(uint8_t c) {
	switch(c) {
		case 0: return !(PIND & (1<<COL_C1));
		case 1: return !(PIND & (1<<COL_C2));
		case 2: return !(PINC & (1<<COL_C3));
		default: return !(PINB & (1<<COL_C4));
	}
}
static char keypad_scan(void) {
	for(uint8_t f=0;f<4;f++) {
		PORTD |= ROW_MASK;
		PORTD &= ~(1<<(4+f));
		_delay_us(5);
		for(uint8_t c=0;c<4;c++) {
			if(col_pressed(c)) {
				_delay_ms(25);
				if(!col_pressed(c)) continue;
				while(col_pressed(c));
				PORTD |= ROW_MASK;
				return teclas[f][c];
			}
		}
	}
	PORTD |= ROW_MASK;
	return 0;
}
static void indicadores_init(void) {
	DDRC |= (1<<LED_VERDE)|(1<<LED_ROJO)|(1<<BUZZER);
	PORTC &= ~((1<<LED_VERDE)|(1<<LED_ROJO)|(1<<BUZZER));
}
static void ok(void) {PORTC|=(1<<LED_VERDE);_delay_ms(500);PORTC&=~(1<<LED_VERDE);}
static void error_led(void) {PORTC|=(1<<LED_ROJO);_delay_ms(500);PORTC&=~(1<<LED_ROJO);}
static void buzz(void) {PORTC|=(1<<BUZZER);_delay_ms(1000);PORTC&=~(1<<BUZZER);}

static void cargar_cuentas(void) {
	/* Si la EEPROM no tiene la tabla actual (primer arranque, chip borrado
	   o tabla por defecto modificada), se copia desde flash. */
	if (eeprom_read_byte(&ee_magic) != EE_MAGIC) {
		uart_print("Inicializando EEPROM con cuentas por defecto\r\n");
		eeprom_update_block(cuentas_def, ee_cuentas, sizeof(cuentas_def));
		eeprom_update_byte(&ee_magic, EE_MAGIC);
	}
	eeprom_read_block(cuentas, ee_cuentas, sizeof(cuentas));
}

static void guardar_saldo(uint8_t idx) {eeprom_update_dword(&ee_cuentas[idx].saldo,cuentas[idx].saldo);}
static uint8_t detectar_tarjeta(uint8_t *uid) {
    memset(uid,0,16);
    mfrc522_standard(uid);
    /* La biblioteca original no devuelve estado ni valida CRC/BCC.
       Este chequeo es solo preliminar; NO autentica una tarjeta. */
    return (uid[0]|uid[1]|uid[2]|uid[3])!=0;
}

static void lcd_num(uint32_t numero) {
    char texto[11];
    sprintf(texto,"%lu",(unsigned long)numero);
    twi_lcd_msg(texto);
}

static void lcd_goto(uint8_t col,uint8_t fila) {
	twi_lcd_cmd((fila ? 0xC0 : 0x80)+col);
}

static void lcd_show(const char *l1,const char *l2) {
	twi_lcd_clear();
	_delay_ms(3);
	lcd_goto(0,0);
	_delay_ms(2);
	twi_lcd_msg(l1);
	if(l2)
	{	lcd_goto(0,1);
		_delay_ms(2);
	twi_lcd_msg(l2);}
}

static uint32_t leer_monto(const char *titulo) {
	char buf[7]; uint8_t i=0;
	lcd_show(titulo,""); lcd_goto(0,1);
	for(;;) {
		char k=keypad_scan();
		if(k>='0'&&k<='9'&&i<6) {buf[i++]=k;twi_lcd_dwr(k);}
		else if(k=='*'&&i) {i--;lcd_goto(i,1);twi_lcd_dwr(' ');lcd_goto(i,1);}
		else if(k=='#'&&i) {
			buf[i]=0;uint32_t v=0;
			for(uint8_t j=0;j<i;j++) v=v*10+(buf[j]-'0');
			return v;
		}
	}
}
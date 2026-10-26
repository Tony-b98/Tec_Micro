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

//  Variables globales

static uint8_t  frame[N_LEDS * 3];          // buffer RGB de la matriz
static uint8_t  hold_mode = 0;              // 1 = HOLD (congelado)
static float    off_roll = 0, off_pitch = 0;// offsets de calibración (°)
static float    bias_gx = 0, bias_gy = 0;   // bias del giroscopio (LSB)
static float    roll_f = 0, pitch_f = 0;    // salida filtro complementario
static float    s_roll = 0, s_pitch = 0;    // salida EMA
static float    roll_out = 0, pitch_out = 0;// ángulos finales (para LCD/UART)
static uint8_t  resync = 1;                 // 1 = reiniciar filtros con el acc
static uint8_t  buzz_ticks = 0;             // buzzer no bloqueante

typedef struct {
    uint8_t row, col;
    uint8_t r, g, b;
    uint8_t at_edge;
} Bubble;
static Bubble bub = {4, 4, 0, 0, 80, 0};

//    PROTOTIPOS

static void twi_write_sla_r(uint8_t sla_r);
static void mpu_write_reg(uint8_t reg, uint8_t val);
static uint8_t mpu_read_reg(uint8_t reg);
static void mpu_init(void);
static void mpu_read(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
static void acc_angles(int16_t ax, int16_t ay, int16_t az, float *roll, float *pitch);

// Matriz WS2812B 

static inline void ws2812_send_byte(uint8_t b);
static void matrix_show(void);
static void matrix_clear(void);
static void matrix_set(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b);

// Buzzer (no bloqueante) 

static void buzzer_init(void);
static void buzz_start(uint16_t ms);
static void buzz_update(void);
static void beep_blocking(uint16_t ms);

//  Botones (polling con debounce) 

static void buttons_init(void);
static uint8_t button_edge(uint8_t bit, uint8_t *prev);

//  EEPROM 

static void eeprom_write_f(float *ee_addr, float f);
static float eeprom_read_f(const float *ee_addr);
static uint8_t valid_f(float f, float lim);

//  LCD / UART 

static void fmt_angle(char *s, float a);
static void build_line1(char *l);
static void build_line2(char *l);
static void lcd_line(uint8_t line, const char *txt);

//  Lógica de la burbuja 

static uint8_t map_axis(float ang);
static void set_color(Bubble *b, float ar, float ap);
static void calibrate(void);

 //IMPLEMENTACIÓN DE LAS FUNCIONES
static void twi_write_sla_r(uint8_t sla_r)
{
    TWDR = sla_r;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    while (TW_STATUS != TW_MR_SLA_ACK);
}

static void mpu_write_reg(uint8_t reg, uint8_t val)
{
    twi_start();
    twi_write_cmd(MPU_ADDR_W);
    twi_write_dwr(reg);
    twi_write_dwr(val);
    twi_stop();
}

static uint8_t mpu_read_reg(uint8_t reg)
{
    uint8_t v;
    twi_start();
    twi_write_cmd(MPU_ADDR_W);
    twi_write_dwr(reg);
    twi_repeated_start();
    twi_write_sla_r(MPU_ADDR_R);
    v = (uint8_t)twi_read_nack();
    twi_stop();
    return v;
}

static void mpu_init(void)
{
    mpu_write_reg(MPU_PWR_MGMT, 0x01);      // despertar, reloj = PLL del gyro X
    _delay_ms(50);
    mpu_write_reg(MPU_SMPLRT_DIV, 0x07);    // 1 kHz / (1+7) = 125 Hz
    mpu_write_reg(MPU_CONFIG,     0x03);    // DLPF ~44 Hz (reduce ruido)
    mpu_write_reg(MPU_GYRO_CFG,   0x00);    // ±250 °/s
    mpu_write_reg(MPU_ACCEL_CFG,  0x00);    // ±2 g
}

// Lee acelerómetro (0x3B..0x40), temperatura y giroscopio (0x43..0x48) en una sola ráfaga de 14 bytes 
static void mpu_read(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    twi_start();
    twi_write_cmd(MPU_ADDR_W);
    twi_write_dwr(MPU_ACCEL_XOUT);
    twi_repeated_start();
    twi_write_sla_r(MPU_ADDR_R);
    for (uint8_t i = 0; i < 13; i++) buf[i] = (uint8_t)twi_read_ack();
    buf[13] = (uint8_t)twi_read_nack();     // último byte: NACK
    twi_stop();

    *ax = (int16_t)((buf[0]  << 8) | buf[1]);
    *ay = (int16_t)((buf[2]  << 8) | buf[3]);
    *az = (int16_t)((buf[4]  << 8) | buf[5]);
    *gx = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// Ángulos por acelerómetro (roll = giro sobre X, pitch = giro sobre Y)
static void acc_angles(int16_t ax, int16_t ay, int16_t az, float *roll, float *pitch)
{
    float fx = ax / ACC_LSB_G, fy = ay / ACC_LSB_G, fz = az / ACC_LSB_G;
    *roll  = atan2f(fy, fz) * RAD_TO_DEG;
    *pitch = atan2f(-fx, sqrtf(fy * fy + fz * fz)) * RAD_TO_DEG;
}

static inline void ws2812_send_byte(uint8_t b)
{
    uint8_t cnt;
    asm volatile (
        "    ldi  %[cnt], 8          \n\t"
        "1:  lsl  %[b]               \n\t"   // 1  carry = bit a enviar
        "    sbi  %[port], %[pin]    \n\t"   // 2  línea en alto
        "    brcc 2f                 \n\t"   // 1/2
        /* ---- bit = 1 ---- */
        NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
        "    cbi  %[port], %[pin]    \n\t"   // 2
        NOP NOP NOP NOP
        "    rjmp 3f                 \n\t"   // 2
        // ---- bit = 0 ---- 
        "2:  nop                     \n\t"
        "    nop                     \n\t"
        "    cbi  %[port], %[pin]    \n\t"   // 2
        NOP NOP NOP NOP NOP NOP NOP
        "3:  dec  %[cnt]             \n\t"   // 1
        "    brne 1b                 \n\t"   // 2
        : [b] "+r" (b), [cnt] "=&d" (cnt)
        : [port] "I" (_SFR_IO_ADDR(WS_PORT)), [pin] "I" (WS_BIT)
    );
}

static void matrix_show(void)
{
    cli();                                  // la trama no admite interrupciones
    for (uint8_t i = 0; i < N_LEDS; i++) {
        ws2812_send_byte(frame[i * 3 + 1]); // orden GRB del WS2812B
        ws2812_send_byte(frame[i * 3 + 0]);
        ws2812_send_byte(frame[i * 3 + 2]);
    }
    sei();
    _delay_us(60);                          // reset/latch: línea en bajo > 50 us
}

static void matrix_clear(void)
{
    for (uint16_t i = 0; i < N_LEDS * 3; i++) frame[i] = 0;
}

// Matriz serpenteante: filas impares invertidas. Ajustar según el módulo. 
static void matrix_set(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b)
{
    if (row > 7 || col > 7) return;
    uint16_t idx;
    if (row & 1) idx = (uint16_t)row * 8 + (7 - col);
    else         idx = (uint16_t)row * 8 + col;
    frame[idx * 3 + 0] = r;
    frame[idx * 3 + 1] = g;
    frame[idx * 3 + 2] = b;
}

//   Buzzer (no bloqueante) 
static void buzzer_init(void)
{
    BUZZ_DDR  |= (1 << BUZZ_BIT);
    BUZZ_PORT &= ~(1 << BUZZ_BIT);
}

static void buzz_start(uint16_t ms)
{
    buzz_ticks = (uint8_t)((ms + LOOP_MS - 1) / LOOP_MS);
    BUZZ_PORT |= (1 << BUZZ_BIT);
}

static void buzz_update(void)               // se llama una vez por lazo
{
    if (buzz_ticks && --buzz_ticks == 0) BUZZ_PORT &= ~(1 << BUZZ_BIT);
}

static void beep_blocking(uint16_t ms)
{
    BUZZ_PORT |= (1 << BUZZ_BIT);
    while (ms--) _delay_ms(1);
    BUZZ_PORT &= ~(1 << BUZZ_BIT);
}

// Botones (polling con debounce) 
static void buttons_init(void)
{
    BTN_DDR  &= ~((1 << BTN_HOLD) | (1 << BTN_CAL));
    BTN_PORT |=  (1 << BTN_HOLD) | (1 << BTN_CAL);  // pull-ups internas
}

// Devuelve 1 sólo en el flanco de pulsación (activo en bajo) 
static uint8_t button_edge(uint8_t bit, uint8_t *prev)
{
    uint8_t now = !(BTN_PIN & (1 << bit));
    if (now && !*prev) {
        _delay_ms(25);                      // debounce
        if (!(BTN_PIN & (1 << bit))) { *prev = 1; return 1; }
        return 0;
    }
    if (!now) *prev = 0;
    return 0;
}

//  EEPROM 
static void eeprom_write_f(float *ee_addr, float f)
{
    eeprom_update_float(ee_addr, f);        // update: sólo escribe si cambió
}

static float eeprom_read_f(const float *ee_addr)
{
    return eeprom_read_float(ee_addr);
}

static uint8_t valid_f(float f, float lim)
{
    return !isnan(f) && fabsf(f) < lim;
}
// LCD / UART 
// Formatea un ángulo como "+12.3" (5 caracteres) sin usar printf de float 
static void fmt_angle(char *s, float a)
{
    if (a >  99.9f) a =  99.9f;
    if (a < -99.9f) a = -99.9f;
    int16_t v = (int16_t)(a * 10.0f + (a >= 0 ? 0.5f : -0.5f));
    s[0] = (v < 0) ? '-' : '+';
    if (v < 0) v = -v;
    s[1] = '0' + v / 100;
    s[2] = '0' + (v / 10) % 10;
    s[3] = '.';
    s[4] = '0' + v % 10;
    s[5] = '\0';
}

// Línea 1: "R:+12.3 P:-04.5 " 
static void build_line1(char *l)
{
    char a[6];
    l[0] = 'R'; l[1] = ':';
    fmt_angle(a, roll_out);
    for (uint8_t i = 0; i < 5; i++) l[2 + i] = a[i];
    l[7] = ' '; l[8] = 'P'; l[9] = ':';
    fmt_angle(a, pitch_out);
    for (uint8_t i = 0; i < 5; i++) l[10 + i] = a[i];
    l[15] = ' '; l[16] = '\0';
}

// Línea 2: "HOLD X:3 Y:7 LIM" / "RUN  X:4 Y:4    " 
static void build_line2(char *l)
{
    const char *m = hold_mode ? "HOLD" : "RUN ";
    for (uint8_t i = 0; i < 4; i++) l[i] = m[i];
    l[4] = ' '; l[5] = 'X'; l[6] = ':'; l[7] = '0' + bub.col;
    l[8] = ' '; l[9] = 'Y'; l[10] = ':'; l[11] = '0' + bub.row;
    l[12] = ' ';
    if (bub.at_edge) { l[13] = 'L'; l[14] = 'I'; l[15] = 'M'; }
    else             { l[13] = ' '; l[14] = ' '; l[15] = ' '; }
    l[16] = '\0';
}

static void lcd_line(uint8_t line, const char *txt)
{
    twi_lcd_cmd(line ? 0xC0 : 0x80);        // inicio de fila 2 / fila 1
    twi_lcd_msg(txt);
}

// Lógica de la burbuja 
// Mapea ángulo [-MAX_ANGLE, +MAX_ANGLE] a [0..7] con saturación 
static uint8_t map_axis(float ang)
{
    if (ang <= -MAX_ANGLE) return 0;
    if (ang >=  MAX_ANGLE) return 7;
    float norm = (ang + MAX_ANGLE) / (2.0f * MAX_ANGLE);   // 0..1
    uint8_t c = (uint8_t)(norm * 8.0f);                    // 8 franjas iguales
    return (c > 7) ? 7 : c;
}

// Color según eje predominante:
 //azul = nivelado | rojo = X (roll) | verde = Y (pitch) | amarillo = combinado 
static void set_color(Bubble *b, float ar, float ap)
{
    if (ar < DEAD_ZONE && ap < DEAD_ZONE) { b->r = 0;   b->g = 0;   b->b = 80; }
    else if (ar > ap * 1.5f)              { b->r = 120; b->g = 0;   b->b = 0;  }
    else if (ap > ar * 1.5f)              { b->r = 0;   b->g = 120; b->b = 0;  }
    else                                  { b->r = 120; b->g = 80;  b->b = 0;  }
}

// Calibración: promedia 64 muestras en reposo y guarda offsets en EEPROM
static void calibrate(void)
{
    matrix_clear();
    matrix_set(3, 3, 60, 60, 60);           // 4 LEDs blancos al centro
    matrix_set(3, 4, 60, 60, 60);
    matrix_set(4, 3, 60, 60, 60);
    matrix_set(4, 4, 60, 60, 60);
    matrix_show();
    lcd_line(0, "Calibrando...   ");
    lcd_line(1, "No mover        ");
    uart_print("Calibrando...\r\n");

    int16_t ax, ay, az, gx, gy, gz;
    float sum_r = 0, sum_p = 0, sum_gx = 0, sum_gy = 0, r, p;

    for (uint8_t i = 0; i < 64; i++) {
        mpu_read(&ax, &ay, &az, &gx, &gy, &gz);
        acc_angles(ax, ay, az, &r, &p);
        sum_r += r;  sum_p += p;
        sum_gx += gx; sum_gy += gy;
        _delay_ms(10);
    }

    off_roll  = sum_r  / 64.0f;
    off_pitch = sum_p  / 64.0f;
    bias_gx   = sum_gx / 64.0f;
    bias_gy   = sum_gy / 64.0f;

    eeprom_write_f(&ee.roll,  off_roll);
    eeprom_write_f(&ee.pitch, off_pitch);
    eeprom_write_f(&ee.gx,    bias_gx);
    eeprom_write_f(&ee.gy,    bias_gy);
    eeprom_update_byte(&ee.magic, MAGIC_OK);

    resync = 1;                             // filtros arrancan del nuevo cero
    beep_blocking(150);                     // confirma calibración
    uart_print("Offset guardado en EEPROM\r\n");
}

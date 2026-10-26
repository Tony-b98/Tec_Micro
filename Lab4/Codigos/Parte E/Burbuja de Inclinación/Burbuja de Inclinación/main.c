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

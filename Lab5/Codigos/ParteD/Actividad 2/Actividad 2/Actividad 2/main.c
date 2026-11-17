// =====================================================
// LABORATORIO 5 - PROBLEMA D - ACTIVIDAD 2
// Impacto de los perifericos mediante PRR
// ATmega328P - ARDUINO UNO - 16 MHz
//
// El micro permanece siempre en Idle. Cada pulsacion del
// boton (D2 / INT0) pasa a la configuracion siguiente:
//   1 parpadeo  : referencia, todos los perifericos habilitados
//   2 parpadeos : sin ADC
//   3 parpadeos : sin USART0
//   4 parpadeos : sin SPI
//   5 parpadeos : sin Timer1
//   6 parpadeos : sin Timer0
//   7 parpadeos : sin Timer2
//   8 parpadeos : sin TWI
//   9 parpadeos : todos deshabilitados
// Despues de los parpadeos el LED queda apagado y el micro
// permanece en esa configuracion hasta la siguiente pulsacion.
//
// El firmware no usa ningun temporizador para funcionar
// (los retardos son por software), por lo que deshabilitar
// Timer0, Timer1 o Timer2 no altera la prueba.
// =====================================================

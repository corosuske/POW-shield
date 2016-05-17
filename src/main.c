// (c) 2015 Cultuurnet Vlaanderen
// Developed by: productize.be

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "uart.h"

//// PINOUT ////

// IN
#define PIN_POWER_OK   PB3 // digital input
#define PIN_BUTTON     PD2 // INT0, falling edge
#define PIN_WAKE       PD3 // INT1, rising edge
#define PIN_SHUTD_BUSY PD5 // digital input

// OUT
#define PIN_RASPI_ON   PA1
#define PIN_NFC_ON     PA0
#define PIN_SHUTDOWN   PD4

// RESERVED
#define PIN_GPIO17     PB2
#define PIN_GPIO18     PB1

// LED
#define PIN_LED0       PD6
#define PIN_LED1       PB4
#define PIN_LED2       PB0

typedef enum {
    MC_BOOT,
    MC_ON,
    MC_WAIT_SHUTDOWN_BUSY,
    MC_WAIT_SHUTDOWN_DONE,
    MC_OFF,
    
} mc_state_t;

static mc_state_t state = MC_BOOT;

static void setup() {
  // Data Direction Register: configure output for select pins
  PORTA = 0;
  DDRA |= _BV(PIN_RASPI_ON) | _BV(PIN_NFC_ON);

  PORTB = 0;
  DDRB |= _BV(PIN_LED1)     | _BV(PIN_LED2) | _BV(PIN_GPIO17) | _BV(PIN_GPIO18);
  DDRB &= ~_BV(PIN_POWER_OK);
  //PORTB &= ~_BV(PIN_POWER_OK); // disable internal pull-up
  PORTB = _BV(PIN_GPIO17) | _BV(PIN_GPIO18);

  PORTD = 0;
  //DDRD = _BV(PIN_LED0) | _BV(PIN_SHUTDOWN);
  DDRD = _BV(PIN_LED0) | _BV(PIN_SHUTDOWN) | _BV(PIN_WAKE);
  //DDRD &= ~(_BV(PIN_BUTTON) | _BV(PIN_WAKE) | _BV(PIN_SHUTD_BUSY));
  //PORTD &= ~(_BV(PIN_WAKE) | _BV(PIN_SHUTDOWN)); // disable
  PORTD = _BV(PIN_BUTTON) | _BV(PIN_SHUTDOWN); // internal pull-up for BUTTON

  // enable pull-ups
  //MCUCR &= ~_BV(PUD);
  // PORTx -> set output
  // PINx -> toggle output
  // configure interrupts
  // level interrupt for both INT0 and INT1
  // as level interrupt is the only kind that can wake
  // from power down
  MCUCR  = 0; // level interrupts, no disable of pull-ups
  GIMSK  = _BV(INT0);  // enable INT0 only; INT1 is only enabled when raspi is on
  //GIMSK = 0;
  
  //init_uart();
  
  //set_sleep_mode(SLEEP_MODE_POWER_DOWN);
  set_sleep_mode(SLEEP_MODE_IDLE);

  // 1 second watchdog timer interrupt (not reset!)
  WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable Watchdog Change Enable
  WDTCSR =   _BV(WDIE) |              // Interupt Mode
             _BV(WDP2) | _BV(WDP1);   // Set Timeout to ~1 seconds
  
  sei(); // enable interrupts
}

static void led0_on() {
  PORTD |= _BV(PIN_LED0);    
}

static void led0_off() {
  PORTD &= ~_BV(PIN_LED0);    
}

static void led0_toggle() {
  PIND |= _BV(PIN_LED0);    
}

static void led1_on() {
  PORTB |= _BV(PIN_LED1);    
}

static void led1_off() {
  PORTB &= ~_BV(PIN_LED1);    
}

static void led1_toggle() {
  PINB |= _BV(PIN_LED1);    
} 

static void led2_on() {
  PORTB |= _BV(PIN_LED2);    
}

static void led2_off() {
  PORTB &= ~_BV(PIN_LED2);    
}

static void led2_toggle() {
  PINB |= _BV(PIN_LED2);    
} 

static void raspi_on() {
  PORTA |= _BV(PIN_RASPI_ON);
  _delay_ms(500);
  // set WAKE as input
  DDRD &= ~_BV(PIN_WAKE);
  PORTD |= _BV(PIN_WAKE); // with active pull-up
  GIMSK  |= _BV(INT1);  // enable INT1 for WAKE
}

static void raspi_off() {
  PORTA &= ~(_BV(PIN_RASPI_ON));
  GIMSK  &= ~_BV(INT1); // disable INT1 for wake
  // set WAKE as OUTPUT
  DDRD |= _BV(PIN_WAKE);
  // and value HIGH
  PORTD |= _BV(PIN_WAKE);
}

static void nfc_on() {
  PORTA |= _BV(PIN_NFC_ON);    
}

static void nfc_off() {
  PORTA &= ~_BV(PIN_NFC_ON);    
}

static void shutdown_on() {
  PORTD &= ~_BV(PIN_SHUTDOWN);    
}

static void shutdown_off() {
  PORTD |= _BV(PIN_SHUTDOWN);    
}

static int is_power_ok() {
  return ((PINB & _BV(PIN_POWER_OK)) != 0);
}

static int is_shutdown_busy() {
  return ((PIND & _BV(PIN_SHUTD_BUSY)) != 0);
}
      
static void main_init() {
  raspi_off(); // need to set IOs in a certain way to allow powering up the raspi
  led0_on();
  _delay_ms(200);
  led1_on();
  _delay_ms(200);
  led0_off();
  led2_on();
  _delay_ms(200);
  led1_off();
  _delay_ms(200);
  led2_off();

  shutdown_off();
  raspi_off();
  nfc_off();
}

static volatile uint64_t seconds = 0;
static volatile uint64_t button_pressed = 0;
static volatile int wake_triggered = 0;
static int shutdown_timeout = 0;

int main() {
  // safety precausion concerning watchdog reset:
  if(MCUSR & _BV(WDRF)) { // If a reset was caused by the Watchdog Timer...
    MCUSR &= ~_BV(WDRF);  // Clear the WDT reset flag
    WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
    WDTCSR = 0x00;        // Disable the WDT
  }

    
  setup();
  //uart_puts("Setup Complete.\r\n");  
  main_init();
  //uart_puts("Init Complete.\r\n");  


  //nfc_on();
  while (1) {
    if (seconds - button_pressed > 3) {
      button_pressed = 0;
    }
    switch (state) {
        
    case MC_BOOT:
        led0_on();
        led1_on();
        led2_on();
        if (is_power_ok()) {
          state = MC_ON;
          raspi_on();
          nfc_on();
        } else {
          state = MC_OFF;
        }
        break;

    case MC_ON:
        led0_toggle();
        led1_off();
        led2_off();
        if (!is_power_ok()) {
          shutdown_on();
          nfc_off();
          _delay_ms(500);
          state = MC_WAIT_SHUTDOWN_BUSY;
          shutdown_timeout = 120;
          break;
        }
        if ((button_pressed > 0) && (seconds - button_pressed < 2)) {
          button_pressed = 0;
          shutdown_on();
          nfc_off();
          _delay_ms(500);
          state = MC_WAIT_SHUTDOWN_BUSY;
          shutdown_timeout = 120;
        }
        break;

    case MC_WAIT_SHUTDOWN_BUSY:
        shutdown_timeout -= 1;
        if (shutdown_timeout <= 0) {
          shutdown_off();
          state = MC_WAIT_SHUTDOWN_DONE;
          break;
        }
        button_pressed = 0;
        led0_off();
        led1_toggle();
        led2_off();
        if (is_shutdown_busy()) {
          shutdown_off();
          state = MC_WAIT_SHUTDOWN_DONE;
        }
        break;

    case MC_WAIT_SHUTDOWN_DONE:
        shutdown_timeout -= 1;
        if (shutdown_timeout <= 0) {
          raspi_off();
          state = MC_OFF;
          break;
        }
        button_pressed = 0;
        led0_off();
        led1_off();
        led2_toggle();
        if (!is_shutdown_busy()) {
          raspi_off();
          state = MC_OFF;
        }
        break;

    case MC_OFF:
        led0_on();
        _delay_ms(10);
        led0_off();
        led1_off();
        led2_off();
        if (button_pressed > 0) {
          button_pressed = 0;
          if (is_power_ok()) {
            raspi_on();
            nfc_on();
            state = MC_ON;
            _delay_ms(500);
            button_pressed = 0;
          }
        }        
    }
    sleep_mode(); // wait for interrupt or UART RX

    /*const uint16_t c = uart_getc();
    if (c == UART_NO_DATA) {
      uart_putc('A');
    }
    else {
      uart_putc(c);
    }*/
    
  }

  return 0;

}

ISR(INT0_vect) {
  if (button_pressed == 0) { button_pressed = seconds; }
}

ISR(INT1_vect) { 
  wake_triggered = 1;
}

ISR(WDT_OVERFLOW_vect) {
  ++seconds;
}

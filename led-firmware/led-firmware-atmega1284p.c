/*
 * led_comm.c
 *
 * Created: 1/9/2019 2:26:54 PM
 * Author : hqss
 */ 
// Microcontroller: atmega1284p 


// Manchester Encoding

#define F_CPU 1000000UL

// Timer calculator: https://eleccelerator.com/avr-timer-calculator/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>


/* Configuration */
#define IR_LED_HIGH     1
#define IR_LED_LOW      0
#define DUTY_CYCLE      10
//#define TMR2_OVF_MAX  112  //the max number of timer2 overflow 112*8 Seconds = 15 minutes Deep sleep
#define TMR2_OVF_MAX  225  //the max number of timer2 overflow 225*8 Seconds = 30 minutes Deep sleep
//#define TMR2_OVF_MAX    3  //the max number of timer2 overflow 3*8 Seconds 24 Seconds Deep sleep
#define TIMER1_TICKS    (65536U - 10000U) // 80ms for between each transition, make 160ms for 1 bit
#define LED_ID          104

// VCC = 1126.4/ADC_value
// ADC_value = 1126.4/VCC 
// Set threshold at 4.3v
// 261.95 = 1126.4/4.3 
#define VOLTAGE_THRESH  262

#define V_MON_DDR DDRA
#define V_MON_PORT PORTA
#define V_MON_PIN PINA
#define V_MON_LED PA3
#define V_MON_CPU_ENn PA4
#define V_MON_MCU PA5

#define LED_DDR DDRB
#define LED_REG PORTB
#define LED_PWR PB2
#define LED_CTRL PB3





/* Configuration */ 

static const uint32_t preamble = 0x01; // {1};
static       uint32_t led_id = LED_ID;
static       uint32_t data;
static       uint16_t index = 0;
static       uint8_t  duty = (255 * DUTY_CYCLE) / 100;
static       uint16_t deep_sleep_timer_overflow_count = 0;
static       uint8_t  in_transmission = 0;
static       uint16_t adc_vcc = 0;
static       uint32_t manchester_clock = 0;

static inline uint16_t calculate_checksum(uint16_t val);
static inline uint16_t  read_adc(void);
static inline void      diable_adc();
static inline void      process_transmission(void);
static inline void      start_transmission(void);
static inline void      init_timer1(void);
static inline void      init_timer2(void);
static inline void      init_adc(void);
static inline void      stop_transmission(void);
static inline void      enter_sleep(void);
static inline void      init_board(void);
static inline void      led_on(void);
static inline void      led_off(void);
static inline void      init_data(void);


static inline void diable_adc() {
  ADMUX = 0x00;
  ADCSRA = 0x00;
  ADCSRB = 0x00;
  DIDR0 = 0x00;
}

static inline uint16_t read_adc(void)
{
  uint16_t adc;
  //start the conversion
  ADCSRA |= _BV(ADSC);
  _delay_ms(2);
  //wait till conversion is complete (ADSC goes back to '0')
  while( ADCSRA & _BV(ADSC) );
  //while (!(ADCSRA & (1 << ADIF)))

  //turn off the ADC after reading
  diable_adc();
  
  // VCC = 1126.4/ADC_value
  adc = ADC;
  return (adc);
}

static inline void init_adc(void)
{
  //init ADMUX, necessary when switching between 2 different config
  ADMUX = 0x00;
  //select AVCC with external capacitor at AREF pin as reference
  ADMUX |= _BV(REFS0);
  //select 1.1V as input
  ADMUX |= (_BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1));
  //left adjust
  //ADMUX |= _BV(ADLAR);
  //enable ADC; prescaller = 16
  ADCSRA |= (_BV(ADEN) | _BV(ADPS2) );

  _delay_ms(5);
}

#define GENPOLY 0x0017 /* x^4 + x^2 + x + 1 */

static inline uint16_t calculate_checksum(uint16_t val)
{
  int i;

  i=1;
  while (val>=16) { /* >= 2^4, so degree(val) >= degree(genpoly) */
    if (((val >> (16-i)) & 1) == 1)
    val ^= GENPOLY << (12-i); /* reduce with GENPOLY */
    i++;
  }
  return val;
}


static inline void init_data(void)
{
  volatile uint32_t voltage_bit = (adc_vcc > VOLTAGE_THRESH);
  
  //led_id = adc_vcc;
  led_id = (uint32_t)LED_ID | (voltage_bit<<15);
  data = 0;
  data |= preamble << 31;
  data |= led_id << 15;
  data |= (0x0F & calculate_checksum(led_id)) << 11;
  
  index = 21;

}

static inline void init_timer1(void)
{
  // set up timer with prescaler = 8
  TCCR1B |= (1 << CS11);
    
  // initialize counter
  TCNT1 = TIMER1_TICKS; /* 65536 - 15000 for 120 milliseconds */
     
  // enable overflow interrupt
  TIMSK1 |= (1 << TOIE1);
    
  sei();
}

static inline void init_timer2(void)
{
  //Disable timer2 interrupts
  TIMSK2 &= ~(_BV(TOIE2) | _BV(OCIE2A) | _BV(OCIE2B));

  //Set clock source to external 32.768KHz crystal
  ASSR |= _BV(AS2);
  TCNT2 = 0;
  TCCR2A = 0;
  //set prescaller to 1024
  //timer2 will overflow every 8 seconds
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  // wait till clock is set
  //while(ASSR & _BV(TCN2UB));
  _delay_ms(10);
  // clear all timer2 interrupts by writing '1'to them
  TIFR2 |= (_BV(OCF2B) | _BV(OCF2A) | _BV(TOV2));

  //wait a bit till clock stabilize
  _delay_ms(1000);
  //Start timer2 interrupt
  TIMSK2 |= _BV(TOIE2);
}

static inline void stop_transmission(void)
{
  TIMSK1 &= ~(1 << OCIE1A); // disable compare match interrupt

  LED_REG &= ~_BV(LED_PWR);  //disable pwr
  
  // reset deep sleep counter
  deep_sleep_timer_overflow_count = 0;
  // Now we are ready to sleep again.
  in_transmission = 0;
}

static inline void start_transmission(void)
{
  index = 0;
  LED_REG |= _BV(LED_PWR);
  led_off();
  in_transmission = 1;
  manchester_clock = 0;
  deep_sleep_timer_overflow_count = 0;  
  init_timer1();
}  


static inline void process_transmission(void)
{
  uint32_t shift = 31 - (uint32_t)index;
  uint32_t data_mask = (uint32_t)1<<shift;
  uint32_t manchester_value = !(!(data & data_mask)) ^ manchester_clock;
  if (index < 21) {
    if (manchester_value) {
      led_on();
    } else {
      led_off();
    }
    if (manchester_clock) {
      index++;
      manchester_clock = 0;
    } else {
      manchester_clock = 1;
    }
  } else {
    led_off();
    stop_transmission();
  }
  
  if ( ( (data ^ (data - 1)) & data ) == data_mask) {
    //measure VCC using internal 1.1V
    init_adc();
    adc_vcc = read_adc();
  }

}


static inline void init_board(void)
{
  V_MON_DDR &= ~(_BV(V_MON_CPU_ENn));           // put it in input mode
  LED_DDR |= _BV(LED_CTRL) | _BV(LED_PWR);      // put B0 & B1 in output mode
  LED_REG &= ~(_BV(LED_CTRL) | _BV(LED_PWR));   // and pull them low to disable LEDs
  V_MON_DDR &= ~(_BV(V_MON_LED));               // set LED Voltage as input
}

static inline void init_pwm(void)
{
  // These bits control the Output Compare pin (OC0A) behavior.
  // Compare Output Mode, Fast PWM Mode Page 103
  TCCR0A |= (1<<COM0A1) | (0<<COM0A0);
  
  // Combined with the WGM02 bit found in the TCCR0B Register,
  // these bits control the counting sequence of the counter
  // Waveform Generation Mode Bit Description: Fast PWM Page 105
  TCCR0A |= (1<<WGM01) | (1<<WGM00);
  
  // Clock Select
  // No prescaling
  // init in disabled mode
  TCCR0B &=  ~(1<<CS00);
  //TCCR0B |=  (1<<CS00);
  
  // Set Timer/Counter Register to zero to reset the output.
  TCNT0 = 0;
  
  DDRB &= ~(1<<PB3);
  
  // Set OCR0A to dutycycle value.
  OCR0A = duty;
}


static inline void led_on(void)
{
  // make sure to make OC0 pin (pin PB3 for atmega32) as output pin
  TCNT0 = 0;
  TCCR0B |=  (1<<CS00);
  DDRB |= (1<<PB3);
}

static inline void led_off(void)
{
  // make sure to make OC0 pin (pin PB3 for atmega32) as output pin
  TCNT0 = 1;
  DDRB &= ~(1<<PB3);
  TCCR0B &=  ~(1<<CS00);
  LED_REG &= ~_BV(LED_CTRL);
}


static inline void enter_sleep(void)
{
  //Set sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  /* Disable unused peripherals */
  power_adc_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer3_disable();
  power_spi_disable();
  power_twi_disable();
  power_usart1_disable();
  //disable interrupt
  cli();
  //enable sleep flag
  sleep_enable();
  //disable BOD
  sleep_bod_disable();
  //enable global interrupt
  sei();
  //Go to sleep!
  sleep_cpu();

  //--- SLEEP ---//

  //wake up by timer2_OVF here
  //disable interrupt
  cli();
  //disable sleep mode
  sleep_disable();
  //reenable global interrupt
  sei();
  power_adc_enable();
  power_timer1_enable();
  init_pwm();
  led_on();
  led_off();
  power_timer0_enable();
}

// TIMER1 overflow interrupt service routine
// called whenever TCNT1 overflows
ISR(TIMER1_OVF_vect)
{
  TCNT1 = TIMER1_TICKS; /* 65536 - 15000 */
  process_transmission();
}

ISR(TIMER2_OVF_vect)
{
  deep_sleep_timer_overflow_count++;
}

int main(int argc, char *argv[])
{
  init_board();
  init_data();
  init_timer2();
  init_pwm();
  led_on();
  led_off();
  sei();
  
  in_transmission = 0;
  deep_sleep_timer_overflow_count = TMR2_OVF_MAX;

  while(1) {
    //see if should go to sleep
    if(in_transmission == 1) {//SleepFlag is not set, (LED is on) not going to sleep
      _delay_ms(50);
    } else { //SleepFlag is set, should go to sleep
      //see if we need to sleep more
      if(deep_sleep_timer_overflow_count < TMR2_OVF_MAX) { //we need to sleep more
        //enter sleep mode
        //_delay_ms(100);
        enter_sleep();
      } else { //we slept enough times, now we do business
        init_data();
        start_transmission();
        //_delay_ms(4000);
      }
    }
  }

  return 0;
}

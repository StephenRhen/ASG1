/**-*-c++-*-
 * 
 * Audio Signal Generator
 * 
 *    Implements a simple audio signal generator capable of producing sine,
 *    square, triangle and sawtooth waves from 2Hz to 20KHz. Amplitude can
 *    be adjusted from 0 to 10 Volts peak.
 *
 * Copyright (c) 2019 Stephen Rhen
 *
 *   This file is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 3, or (at your option)
 *   any later version.
 */
 
#include <PinChangeInterrupt.h>
#include <LiquidCrystal_I2C.h>

/* Pin assignments */
#define FREQ_ENC_CLK       8
#define FREQ_ENC_DATA      9
#define PW_ENC_CLK        11
#define PW_ENC_DATA       10
#define WAVE_BUTTON       A3
#define OUT_VOLTAGE       A0
#define OUT_RANGE         A1
#define RANGE_BUTTON      12

#define FREQ_ENC_MIN     2
#define FREQ_ENC_MAX   200

#define PW_ENC_MIN       1
#define PW_ENC_MAX      99

/* Range multiplier values */
#define  RANGE_200    1     /* 2Hz to 200Hz, resolution 2Hz */
#define  RANGE_2K     2     /* 0.02KHz to 2.00Khz, resolution 20Hz  */
#define  RANGE_20K    3     /* 0.2KHz to 20.0KHz, resolution 200hz */

/* Waveforms */
#define SINE      0
#define SAW       1
#define TRIANGLE  2
#define SQUARE    3
#define PULSE     4

#define VOLT_RANGE_1V  0
#define VOLT_RANVE_10V 1

const char *wavename[] = { "Sine", "Saw", "Triangle", "Square", "Pulse" };

extern const long sine_table_size;
extern const uint8_t sine_table[];

uint32_t clock_frequency = 16000000;   // 16MHz clock
uint32_t sample_rate = 131072;         // 131KHz sample rate

/* Variables used by the waveform generator interrupt */
uint16_t step = 100;
uint16_t duty_cycle = 0x8000; 

/* State variables for the controls */
uint8_t freq = FREQ_ENC_MIN;
uint8_t range = RANGE_200;
uint8_t pulse_width = 50;
uint8_t waveform = SINE;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);


void setup()
{

  lcd.init();
  lcd.backlight();

  /* set port Port D, pins 0 through 7 as output  
   *  for the DAC.
   */
  DDRD = 0xff;

  pinMode(FREQ_ENC_CLK, INPUT);
  pinMode(FREQ_ENC_DATA, INPUT);
  pinMode(PW_ENC_CLK, INPUT);
  pinMode(PW_ENC_DATA, INPUT);
  pinMode(RANGE_BUTTON, INPUT);
  pinMode(WAVE_BUTTON,INPUT);
  pinMode(OUT_RANGE, INPUT_PULLUP);

  /* Set 16 bit timer 1 to generate a periodic interrupt  when 
   *  it matches a preset value.
   */
  cli();                         /* Disable interrupts */
  TCCR1A = 0;                    /* Clear the control registers */
  TCCR1B = 0;
                                 /* Count to match */
  OCR1A = clock_frequency/sample_rate - 1;
  TCCR1B |= (1 << WGM12);        /* Clear timer on match */
  TCCR1B |= (1 << CS10);         /* No pre-scaler */
  TIMSK1 |= (1 << OCIE1A);       /* Enable timer compare interrupt. */


  attachPCINT(digitalPinToPCINT(FREQ_ENC_CLK), freq_ctrl_isr, RISING);
  attachPCINT(digitalPinToPCINT(PW_ENC_CLK), pw_ctrl_isr, RISING);
  attachPCINT(digitalPinToPCINT(RANGE_BUTTON), range_button_isr, FALLING);
  attachPCINT(digitalPinToPCINT(WAVE_BUTTON), wave_button_isr, FALLING);

  sei();                         /* Re-enable interrupts. */   
  
}

void loop()
{
  static uint8_t last_freq = 0;
  static uint8_t last_range = 0;
  static uint8_t last_waveform = 0xff;
  static uint8_t last_pw = 0;
  static int last_voltage = 99;
  static int last_volt_range = 99;

  char str[16];

  if ((waveform != last_waveform) || (pulse_width != last_pw)) {
    last_waveform = waveform;
    last_pw = pulse_width;
    
    /* Update the duty cycle */
    uint16_t dc = 0x8000;
    if (last_waveform == PULSE) {
      dc = (0xffff/100) * pulse_width;
    }

    cli();
    duty_cycle = dc;
    sei();
    
    lcd.setCursor(0,1);
    lcd.print(wavename[last_waveform]);

    if (last_waveform == PULSE)
      snprintf(str, sizeof(str), "%-8s %2u%%", wavename[last_waveform], pulse_width);
    else
      snprintf(str, sizeof(str), "%-12s", wavename[last_waveform]);

    lcd.setCursor(0,1);
    lcd.print(str);
  }

  if((freq != last_freq) || (range != last_range)) {
   
    last_freq = freq;
    last_range = range;

    cli();
    step = last_freq * 10^last_range;
    sei();

    switch (last_range) {
      case RANGE_200:
        snprintf(str, sizeof(str), "%4u Hz ", last_freq);
        break;

      case RANGE_2K:
        snprintf(str, sizeof(str), "%1u.%02u KHz", last_freq/100, last_freq % 100);
        break;

      case RANGE_20K:
        snprintf(str, sizeof(str), "%2u.%1u KHz", last_freq/10, last_freq % 10);
        break; 
    }

    lcd.setCursor(0,0);
    lcd.print(str);    
    
  }

  long voltage = analogRead(OUT_VOLTAGE);
  int voltage_range = digitalRead(OUT_RANGE);

  if ((voltage != last_voltage) || (voltage_range != last_volt_range)) {
    voltage = map(voltage, 0, 1024, 0, 100);
    if (voltage_range == VOLT_RANGE_1V) {
      snprintf(str, sizeof(str), "%1u.%02u V", voltage/100, last_freq % 100);
    }
    else {
      snprintf(str, sizeof(str), "%2u.%1u V", voltage/10, voltage % 10);
    }

    lcd.setCursor(10,0);
    lcd.print(str);
  }
  
  delay(200);

}

/*  
 *   Timer interrupt routine.
 *   
 *   Top 10 bits of the index are the offset into the 1024
 *   entry sine lookup table. The lower bits improve accuracy
 *   and allow a single table entry to read multiple successive 
 *   times when operating at a low frequency. The offset is
 *   incremented by sine_step on every iteration and allowed
 *   to wraps so it doesn't exceed the end of table.
 */
ISR(TIMER1_COMPA_vect)
{
  static uint16_t index = 0;
  uint8_t value;
 
  index += step;

  switch (waveform) {

    case SINE:
      value = pgm_read_byte_near(sine_table + (index >> 6));
      break;

    case SAW:
      value = 255 - (index >> 8);
      break;

    case TRIANGLE:
       value = index >> 7;
       if (index > 0x8000)
         value = 255 - value;
       break;

    case PULSE:          
    case SQUARE:
      value = (index > duty_cycle) ? 225 : 0;
      break;
      
  }
  
  PORTD = value;
}

void freq_ctrl_isr()
{
  if (digitalRead(FREQ_ENC_DATA)) {
    if (freq > FREQ_ENC_MIN)
      freq--;
  }
  else if (freq < FREQ_ENC_MAX)
    freq++;
}

void pw_ctrl_isr()
{
  if (digitalRead(PW_ENC_DATA)) {
    if (pulse_width > PW_ENC_MIN)
      pulse_width--;
  }
  else if (pulse_width < PW_ENC_MAX)
    pulse_width++;
}

void  range_button_isr()
{
  range++;
  if (range > RANGE_20K)
    range = RANGE_200;
}


void wave_button_isr()
{
  waveform++;
  if (waveform > PULSE)
    waveform = SINE;
}

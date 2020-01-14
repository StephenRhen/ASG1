/*
 * Simple Rotary Encoder example using pin change interrupts.
 */
#include "PinChangeInterrupt.h"

#define ENCODER1_CLK     8
#define ENCODER1_DATA    9
#define ENCODER1_BUTTON  2

#define ENCODER2_CLK     11
#define ENCODER2_DATA    10
#define ENCODER2_BUTTON   3

#define ENCODER_MIN_VALUE   1
#define ENCODER_MAX_VALUE 255

/* Reads and writes to BYTE types are atomic, so they can
 * be safely shared with interrupt handlers. 
 */
volatile uint8_t encoder1_value = ENCODER_MIN_VALUE;
volatile uint8_t encoder1_button = 0;

void encoder1_isr()
{
  if (digitalRead(ENCODER1_DATA)) {
    if (encoder1_value > ENCODER_MIN_VALUE)
      encoder1_value--;
  }
  else if (encoder1_value < ENCODER_MAX_VALUE)
    encoder1_value++;
}

void encoder1_button_isr() { encoder1_button++; }

volatile uint8_t encoder2_value = ENCODER_MIN_VALUE;
volatile uint8_t encoder2_button = 0;

void encoder2_isr()
{
  if (digitalRead(ENCODER2_DATA)) {
    if (encoder2_value > ENCODER_MIN_VALUE)
      encoder2_value--;
  }
  else if (encoder2_value < ENCODER_MAX_VALUE)
    encoder2_value++;
}

void encoder2_button_isr() { encoder2_button++; }

void setup()
{
  Serial.begin(9600);
  Serial.println("Encoder Interrupt Test:");

  pinMode(ENCODER1_CLK, INPUT);
  pinMode(ENCODER1_DATA, INPUT);
  pinMode(ENCODER1_BUTTON, INPUT);

  attachPCINT(digitalPinToPCINT(ENCODER1_CLK),
              encoder1_isr, RISING);
  attachPCINT(digitalPinToPCINT(ENCODER1_BUTTON),
              encoder1_button_isr, FALLING);

  pinMode(ENCODER2_CLK, INPUT);
  pinMode(ENCODER2_DATA, INPUT);
  pinMode(ENCODER2_BUTTON, INPUT);

  attachPCINT(digitalPinToPCINT(ENCODER2_CLK),
              encoder2_isr, RISING);
  attachPCINT(digitalPinToPCINT(ENCODER2_BUTTON),
              encoder2_button_isr, FALLING);
}

void loop()
{
  static int encoder1_last = 0;
  static int encoder2_last = 0;  

  if (encoder1_value != encoder1_last) {
    Serial.print("Enc1: ");
    Serial.println(encoder1_value);
    encoder1_last = encoder1_value;
  }
  
  while (encoder1_button) {
    cli();
    encoder1_button--;
    sei();
    Serial.println("Button1");
  }

  if (encoder2_value != encoder2_last) {
    Serial.print("Enc2: ");
    Serial.println(encoder2_value);
    encoder2_last = encoder2_value;
  }
  
  while (encoder2_button) {
    cli();
    encoder2_button--;
    sei();
    Serial.println("Button2");
  }

  delay(100);
}

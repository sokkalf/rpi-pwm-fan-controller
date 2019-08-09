#define I2C_SLAVE_ADDRESS 0x2A

#include <Arduino.h>
#include <TinyWireS.h>

#include <util/delay.h>

// 8MHz clock
#define F_CPU 8000000

const int PWMPin = PB1;
const int PotPin = A1;
const int PwrPin = 3;

#define EVENT_READ_FAN 0x01
#define EVENT_SET_FAN 0x02

volatile uint8_t Count;
uint8_t rps = 0;

long previousMillis = 0; 
long interval = 26000; // clock is way off because of use of timer0 for pwm. 26000 ~= 1 second

// pin change interrupt, increment count when sense pin changes (fan speed)
ISR(PCINT0_vect) {
  Count++; 
}

void requestEvent()
{
  TinyWireS.write(rps);
}

void receiveEvent(uint8_t size)
{
  uint8_t fan_speed;
  if(size > 1) {
    if(TinyWireS.read() == EVENT_SET_FAN) {
      fan_speed = TinyWireS.read();
      set_fan_speed(fan_speed);
    }
  }
}

void setup()
{
  pinMode(PWMPin, OUTPUT);
  // Phase Correct PWM Mode, no Prescaler
  // PWM on Pin 1(PB1), Pin 0(PB0) disabled
  // 8Mhz / 160 / 2 = 25Khz
  TCCR0A = _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(WGM02) | _BV(CS00); 
  // Set TOP and initialize duty cycle to zero(0)
  OCR0A = 160;  // TOP - DO NOT CHANGE, SETS PWM PULSE RATE
  OCR0B = 0;    // duty cycle for Pin 1(PB1)

  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onRequest(requestEvent);
  TinyWireS.onReceive(receiveEvent);

  // enable pin change interrupt for reading fan speed (sense pin)
  GIMSK = _BV(PCIE);    // Enable pin change interrupt Table 9.3.2
  PCMSK = _BV(PCINT4);  // Enable the interrupt for only pin 4,Table 9.3.4

  set_fan_speed(30);
  pinMode(PwrPin, OUTPUT);
}

void set_fan_speed(uint8_t speed)
{
  if(speed >= 20) {
    turn_fan_on();
    OCR0B = speed;
  } else {
    OCR0B = 0;
    turn_fan_off();
  }
}

void turn_fan_off() {
  digitalWrite(PwrPin, LOW);
}

void turn_fan_on() {
  digitalWrite(PwrPin, HIGH);
}


void loop()
{
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;   
    rps = Count;
    Count=0;
  }
}

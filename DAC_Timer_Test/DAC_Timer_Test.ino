#include <SPI.h>         // Remember this line!
#include <DAC_MCP49xx.h>
#include "Lookup.h"

// The Arduino pin used for the slave select / chip select
#define SS_PIN 10

// Set up the DAC. 
// First argument: DAC model (MCP4901, MCP4911, MCP4921)
// Second argument: SS pin (10 is preferred)
// (The third argument, the LDAC pin, can be left out if not used)

//double pi = 3.141592;
//int value;
int values[samples];
int count = 0;

DAC_MCP49xx dac(DAC_MCP49xx::MCP4921, SS_PIN,7);


void setup() {
  
  dac.setSPIDivider(SPI_CLOCK_DIV2);
  //pinMode(6,1);
  //Serial.begin(9600);
  for(int i=0;i<samples;i++)
  {
    values[i] = sine[i];
  }
  dac.setPortWrite(true);
  //dac.bufferVref(true);
  cli();
  //set timer0 interrupt at 2kHz
  TCCR2A = 0;// set entire TCCR0A register to 0
  TCCR2B = 0;// same for TCCR0B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 100 khz increments
  OCR2A = 24;//49;// 16*10^6/(100k*64);
  // turn on  CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS01 and CS00 bits for 32 prescaler
  TCCR2B |= (1<<CS20);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();
  //TIMSK0=0;
}

ISR(TIMER2_COMPA_vect){//timer0 interrupt 125kHz toggles pin 8
//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
  //dac.output(values[count]);
  dac.output(4095*count%2);
  dac.latch();
  //digitalWrite(6,count%2);
  count++;
  if(count == samples)
    count = 0;
}

void loop() {
  // put your main code here, to run repeatedly:

}

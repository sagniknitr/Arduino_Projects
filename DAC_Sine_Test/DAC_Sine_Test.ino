//
// Example for the MCP49x1 *single* DACs
// For the dual MCP49x2 series, see the other bundled example sketch.
//

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

DAC_MCP49xx dac(DAC_MCP49xx::MCP4921, SS_PIN);

void setup() {
  // Set the SPI frequency to 1 MHz (on 16 MHz Arduinos), to be safe.
  // DIV2 = 8 MHz works for me, though, even on a breadboard.
  // This is not strictly required, as there is a default setting.
  dac.setSPIDivider(SPI_CLOCK_DIV2);
  Serial.begin(9600);
  for(int i = 0;i<samples;i++)
  {
    values[i] = sine[i];
  }
  // Use "port writes", see the manual page. In short, if you use pin 10 for
  // SS (and pin 7 for LDAC, if used), this is much faster.
  // Also not strictly required (no setup() code is needed at all).
  dac.setPortWrite(true);
}

// Output something slow enough that a multimeter can pick it up.
// For MCP4901, use values below (but including) 255.
// For MCP4911, use values below (but including) 1023.
// For MCP4921, use values below (but including) 4095.
void loop() {
 // dac.output(4095);
 // delay(1000000);
  //delayMicroseconds(10);
  //dac.output(0);
  //delayMicroseconds(10);
  /*for(int i = 0;i<4096;i++)
  {
    dac.output(i);
    delay(1);
  }
  delay(2000);
  for(int i = 4095;i>=0;i--)
  {
    dac.output(i);
    delay(1);
  }*/
  //cli();
  //TIMSK0 = 0;
  //noInterrupts();
  while(1)
  {
    for(int i=0;i<samples;i++)
    {
      dac.output(values[i]);
      dac.latch();
      //delay(2);
      delayMicroseconds(10);
      //Serial.println(String(i)+ " " + String(3.3*values[i]/4095));
    }
  }
    //delay(100000);
}

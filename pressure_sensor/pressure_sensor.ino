#include <Wire.h>


//Commands
#define CMD_RESET 0x1E // reset command 
#define CMD_ADC_READ 0x00 // ADC read command 
#define CMD_ADC_CONV 0x40 // ADC conversion command 

#define CMD_PROM 0xA0 // Coefficient location


    
enum temperature_units
{
  CELSIUS,
  FAHRENHEIT,
};
temparature_units _temperature_unit=CELSIUS;

// Define measurement type.
enum measurement
{  
  PRESSURE = 0x00,
  TEMPERATURE = 0x10
  };
measurement _measurement=PRESSURE;
// Define constants for Conversion precision
enum precision
{
  ADC_256  = 0x00,
  ADC_512  = 0x02,
  ADC_1024 = 0x04,
  ADC_2048 = 0x06,
  ADC_4096 = 0x08
};
precision _precision=ADC_256;
// Define address choices for the device (I2C mode)
enum ms5803_addr
{
  ADDRESS_HIGH = 0x76,
  ADDRESS_LOW  = 0x77
};

ms5803_addr _address;

//Create variables to store results
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;


// Create Variable to store altitude in (m) for calculations;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)

int32_t _temperature_actual;
int32_t _pressure_actual;

int coefficients[8];

void setup() {
  // put your setup code here, to run once:
Wire.begin();
Serial.begin(9600);
_begin();
}


void sendCommand(uint8_t command)
{  
  Wire.beginTransmission(_address);//adress);
  Wire.write(command);
  Wire.endTransmission();
  
}



void reset(void)
// Reset device I2C
{
   sendCommand(CMD_RESET);
   sensorWait(3);
}

uint32_t getADCconversion(measurement _measurement, precision _precision)
// Retrieve ADC measurement from the device.  
// Select measurement type and precision
{  
  uint32_t result;
  uint8_t highbyte, midByte, lowbyte;
  
  sendCommand(CMD_ADC_CONV + _measurement + _precision);
  // Wait for conversion to complete
  sensorWait(1); //general delay
  switch( _precision )
  { 
    case ADC_256 : sensorWait(1); break; 
    case ADC_512 : sensorWait(3); break; 
    case ADC_1024: sensorWait(4); break; 
    case ADC_2048: sensorWait(6); break; 
    case ADC_4096: sensorWait(10); break; 
  } 
  
  sendCommand(CMD_ADC_READ);
  Wire.requestFrom(_address, 3);
  
  while(Wire.available())    
  { 
    highbyte = Wire.read();
    midByte = Wire.read();
    lowbyte = Wire.read();  
  }
  
  result = ((uint32_t)highbyte << 16) + ((uint32_t)midByte << 8) + lowbyte;

  return result;

}








void getMeasurements(precision _precision)

{
  //Retrieve ADC result
  int32_t temperature_raw = getADCconversion(TEMPERATURE, _precision);
  int32_t pressure_raw = getADCconversion(PRESSURE, _precision);
  
  
  //Create Variables for calculations
  int32_t temp_calc;
  int32_t pressure_calc;
  
  int32_t dT;
    
  //Now that we have a raw temperature, let's compute our actual.
  dT = temperature_raw - ((int32_t)coefficient[5] << 8);
  temp_calc = (((int64_t)dT * coefficient[6]) >> 23) + 2000;
  
  // TODO TESTING  _temperature_actual = temp_calc;
  
  //Now we have our first order Temperature, let's calculate the second order.
  int64_t T2, OFF2, SENS2, OFF, SENS; //working variables

  if (temp_calc < 2000) 
  // If temp_calc is below 20.0C
  { 
    T2 = 3 * (((int64_t)dT * dT) >> 33);
    OFF2 = 3 * ((temp_calc - 2000) * (temp_calc - 2000)) / 2;
    SENS2 = 5 * ((temp_calc - 2000) * (temp_calc - 2000)) / 8;
    
    if(temp_calc < -1500)
    // If temp_calc is below -15.0C 
    {
      OFF2 = OFF2 + 7 * ((temp_calc + 1500) * (temp_calc + 1500));
      SENS2 = SENS2 + 4 * ((temp_calc + 1500) * (temp_calc + 1500));
    }
    } 
  else
  // If temp_calc is above 20.0C
  { 
    T2 = 7 * ((uint64_t)dT * dT)/pow(2,37);
    OFF2 = ((temp_calc - 2000) * (temp_calc - 2000)) / 16;
    SENS2 = 0;
  }
  
  // Now bring it all together to apply offsets 
  
  OFF = ((int64_t)coefficient[2] << 16) + (((coefficient[4] * (int64_t)dT)) >> 7);
  SENS = ((int64_t)coefficient[1] << 15) + (((coefficient[3] * (int64_t)dT)) >> 8);
  
  temp_calc = temp_calc - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

  // Now lets calculate the pressure
  

  pressure_calc = (((SENS * pressure_raw) / 2097152 ) - OFF) / 32768;
  
  _temperature_actual = temp_calc ;
  _pressure_actual = pressure_calc ; // 10;// pressure_calc;
  

}





uint8_t _begin(void)
// Initialize library for subsequent pressure measurements
{  
  uint8_t i;
  for(i = 0; i <= 7; i++){
    sendCommand(CMD_PROM + (i * 2));
    Wire.requestFrom( _address, 2);
    uint8_t highbyte = Wire.read(); 
    uint8_t lowbyte = Wire.read();
    coefficient[i] = (highbyte << 8)|lowbyte;
  // Uncomment below for debugging output.
  //  Serial.print("C");
  //  Serial.print(i);
  //  Serial.print("= ");
  //  Serial.println(coefficient[i]);
  }

  return 0;
}

float getTemperature(temperature_units units, precision _precision)
// Return a temperature reading in either F or C.
{
  getMeasurements(_precision);
  float temperature_reported;
  // If Fahrenheit is selected return the temperature converted to F
  if(units == FAHRENHEIT){
    temperature_reported = _temperature_actual / 100;
    temperature_reported = (((temperature_reported) * 9) / 5) + 32;
    return temperature_reported;
    }
    
  // If Celsius is selected return the temperature converted to C 
  else {
    temperature_reported = _temperature_actual / 100;
    return temperature_reported;
  }
}








float getPressure(precision _precision)
// Return a pressure reading units Pa.
{
  getMeasurements(_precision);
  float pressure_reported;
  pressure_reported = _pressure_actual;
  pressure_reported = pressure_reported / 10;
  return pressure_reported;
}



void sensorWait(uint8_t time)
// Delay function.  This can be modified to work outside of Arduino based MCU's
{
  delay(time);
}






void loop() {
  // put your main code here, to run repeatedly:


  // Read pressure from the sensor in mbar.
  pressure_abs = getPressure(ADC_4096);

  Serial.print("pressure_data");
  Serial.println(pressure_abs);
delay(1000);

}





int cs = 53;
int miso = 50;
int mosi = 51;
int sck = 52;
int readValue = 0;
float volts = 0.0;

#define timer 300
int values[timer];
void setup() 
{
  // put your setup code here, to run once:
  pinMode(cs,OUTPUT);
  digitalWrite(cs, HIGH);
  pinMode(miso,INPUT);
  pinMode(mosi,OUTPUT);
  pinMode(sck,OUTPUT);
  Serial.begin(115200);
  SPCR = 0x00;
  SPCR |= (1<<SPE)|(1<<MSTR);//|(1<<SPR0);//|(1<<SPI2X);
  sei();
}

void loop() 
{
  //while(!(Serial.available()));
  char c = 's';//Serial.read();
  if(c == 's')
  {
    Serial.print("Sampling starts at : ");
    long starttime = micros();
    for(int i = 0;i<timer;i++)
    {
     // values[i] = adcRead(2);
     Serial.println("inside for loop");
       values[i] = adcRead(3);
       values[i] = (float)readValue*4.83/4096.0;
      Serial.println(values[i]);
    //  values[i] = volts;
    }
    long endtime = micros();
    Serial.println(starttime);
    Serial.println("Ends at : " + String(endtime));
    Serial.println("Time taken : " + String(endtime - starttime) + " microseconds for 300 samples");
    Serial.println("Values are : ");
    for(int i = 0;i<timer;i++)
    {
      
       Serial.println((float)values[i]*4.83/4096.0);
    }
  }
  //int readValue = adcRead(1);
  //float volts = (float)readValue*4.83/4096.0;
  //Serial.println(volts);
  Serial.flush();
}
int adcRead(int adcnumber)
{
  if(adcnumber >4 || adcnumber <1)
     adcnumber = 1;
//  unsigned char control = 0x00;
//  if((adcnumber-1)/2)
//    control = 1 << 4;
//  if(((adcnumber-1)%2))
//    control = 1 << 3;
  PORTB &= !(1<<0);  // cs pin on pin 10 = PB2  PB4 on mega
  
  unsigned char top8 = spi_transceiver((adcnumber-1) << 3);
  unsigned char bottom8 = spi_transceiver(0x00);
  PORTB |= (1<<0);  
  int value = (top8*256 + bottom8)%4096;// & 0x0FFF;
  return value;
  
}
unsigned char spi_transceiver (unsigned char data)
{
    // Load data into the buffer
    SPDR = data;

    //Wait until transmission complete
    while(!(SPSR & (1<<SPIF) ));
    //for(int a = 0;a<8;a++)
    //{}
    // Return received data
    return(SPDR);
}

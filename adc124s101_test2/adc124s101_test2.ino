int cs = 10;
int miso = 12;
int mosi = 11;
int sck = 13;

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
  SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR0);
  sei();
}

void loop() 
{
  int readValue = adcRead(1);
  float volts = (float)readValue*4.83/4096.0;
  Serial.println(volts);
  delay(1000);
}
int adcRead(int adcnumber)
{
  if(adcnumber >4 || adcnumber <1)
     adcnumber = 1;
  int control = 0x00;
  if((adcnumber-1)/2)
    control = 1 << 4;
  if(((adcnumber-1)%2))
    control = 1 << 3;
  PORTB &= !(1<<2);  // cs pin on pin 10 = PB2
  
  unsigned char top8 = spi_transceiver(0x00);
  unsigned char bottom8 = spi_transceiver(0x00);
  PORTB |= (1<<2);  
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

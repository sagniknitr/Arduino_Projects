int fsync = 9;
//SCLK is connected to 13, SDATA to 11
//Generate a 2 MHz clock - OC0A - Pin 6 and connect to MCLK 
//Perform SPI at 125 KHz also try at 1 MHz
void setup() 
{
  Serial.begin(9600);
  pinMode(fsync,OUTPUT);
  digitalWrite(fsync,HIGH);
  pinMode(13,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(10,OUTPUT);
  TCCR0B = 0; 
  TCCR0A = 0;
  TCCR0A |= (1<<WGM01)|(0<<WGM00);
  TCCR0A |= (0<<COM0A1)|(1<<COM0A0);
  TCCR0B |= (1<<CS01);
  OCR0A = 1;
  TCNT0 = 0;
  SPCR = 0x00;
  SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0)|(1<<CPOL);
  sei();
  
  //delay(100);
  
  //digitalWrite(fsync,LOW);
  
  unsigned char dataMSB = (1<<5)|(1<<0); // B28 and RESET
  unsigned char dataLSB = 0;
  Serial.println(dataMSB);
  PORTB &= !(0x06);
  spi_transmit(dataMSB);
  spi_transmit(dataLSB);
  PORTB |= (0x06);
  //Serial.print("Here");
  
  dataMSB = 0x7F;       // Write freq register
  dataLSB = 0;
  PORTB &= !(0x06);
  spi_transmit(dataMSB);
  spi_transmit(dataLSB);
  PORTB |= (0x06);
  dataMSB = 0x7F;
  dataLSB = 0;
  PORTB &= !(0x06);
  spi_transmit(dataMSB);        // 2nd word of freq register
  spi_transmit(dataLSB);
  PORTB |= (0x06);
  dataMSB = (1<<7)|(1<<6);
  dataLSB = 0;
  PORTB &= !(0x06);
  spi_transmit(dataMSB);
  spi_transmit(dataLSB);
  PORTB |= (0x06);
  dataMSB = (1<<5);
  dataLSB = 0;
  PORTB &= !(0x06);
  spi_transmit(dataMSB);
  spi_transmit(dataLSB);
  PORTB |= (0x06);
  
  Serial.print("End");
  
  /* while(1)
   {
     dataMSB = (1<<5)|(1<<2);
     dataLSB =0;
      PORTB &= !(0x06);
  spi_transmit(dataMSB);
  spi_transmit(dataLSB);
  PORTB |= (0x06);
   }*/
   //digitalWrite(fsync,HIGH);
}

unsigned char spi_transmit(unsigned char data)
{
  SPDR = data;
  //Wait until transmission complete
  while(!(SPSR & (1<<SPIF) ));
  data = SPDR;
}

void loop() {
  // put your main code here, to run repeatedly:
}

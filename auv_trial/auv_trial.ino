void setup() {
  // put your setup code here, to run once:
  TCCR0B = 0; 
  TCCR0A = 0;
  TCCR0A |= (1<<WGM01)|(0<<WGM00);
  TCCR0A |= (0<<COM0A1)|(1<<COM0A0);
  TCCR0B |= (1<<CS01);
  OCR0A = 1;
  TCNT0 = 0;
  SPCR = 0x00;
  SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0)|(1<<CPOL);
   SPCR = 0x00;
  SPCR |= (1<<SPE)|(1<<MSTR);//|(1<<SPR0);//|(1<<SPI2X);
  sei();

}

void loop() {
  // put your main code here, to run repeatedly:
PORTB=!();

}

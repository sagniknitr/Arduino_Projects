
char Path[200];
float Time[200];
int turns=0;


void setup() {
  // put your setup code here, to run once:
  DDRA=B00000000;    // pins D0-D7 as input
  //DDRB=B00001111  ;   //dugutil pins 8-11 as output
   //pinMode(3, OUTPUT);
  //pinMode(11, OUTPUT);
  //TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  //TCCR2B = _BV(WGM22) | _BV(CS22);
  //OCR2A = 180;
  //OCR2B = 50;
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  PINA;
  float timer=micros();

   
  if(!(PINA|B01111100==B01110100))
  {
    //forward
    Serial.println("jhjh");
  }
  else if(PIND|B00001110==B00001110)
  {
    //left
    Path[turns]='L';
    
    
  }
  else if(PIND|B00111000==B00111000)
  {
    //right
    Path[turns]='R';
  }
  else if(PIND|B00011100==B00011100)
  {
    
  }
 
  else if(PIND|B00111110==B00000000)
  {
      //turn back
      Path[turns]='W';
  }
  else
  {
    Serial.println("dddddddddd");
  }
  turns++;
  
}

void forward()
{
  PORTB=B00001010;
}
void backward()
{
  PORTB=B00000101;
  
}
void right()
{
  PORTB=B00001001;
}
void left()
{
  PORTB=B00000110;
}

void shortest()
{
  for(int i=0;i<=turns;i++)
  {
  
    
  }
  
}



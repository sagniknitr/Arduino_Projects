
char Path[200];
float Time[200];
int turns=0;


void setup() {
  // put your setup code here, to run once:
  DDRD=B00000000;    // pins D0-D7 as input
  //DDRB=B00001111     //dugutil pins 8-11 as output
    Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  PIND;
  float timer=micros();

  if("calculate distance from IR sensor")
  {  
  if((PIND|B01111100==B01111100)
  {
    //forward
    Serial.println("gchf");
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
  turns++;
  }
  else
  {
    //Stop and light LED
  }

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



int cs = 53;
int miso = 50;
int mosi = 51;
int sck = 52;

void setup() {
  // put your setup code here, to run once:
  pinMode(cs,OUTPUT);
  pinMode(miso,INPUT);
  pinMode(mosi,OUTPUT);
  pinMode(sck,OUTPUT);
  Serial.begin(115200);
  digitalWrite(cs, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  int readValue = adcRead(1);
  float volts = (float)readValue*4.83/4096.0;
  Serial.println(volts); 
  
  delay(1000);
}
int adcRead(int adcnumber)
{
  int value = 0;
  if(adcnumber >4 || adcnumber <1)
     adcnumber = 1; 
  digitalWrite(sck, LOW);
  digitalWrite(cs, LOW);
  for(int i = 0; i < 16 ;i++)
  {
    digitalWrite(sck,HIGH);
    delayMicroseconds(10);
    digitalWrite(sck,LOW);
//    if( i == 3)
//    {
//        digitalWrite(mosi,(adcnumber-1)/2);
//    }
//    else if( i == 4 )
//    { 
//        digitalWrite(mosi,(adcnumber-1)%2);
//    }
    if(i>3)
    {
      value*=2;
      if(digitalRead(miso))
      {
       // Serial.println(i);
        value |= 0x01;
      }
    }
  }
  digitalWrite(cs,HIGH);
  return value;
}

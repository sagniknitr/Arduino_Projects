#define encoderPinA  2

volatile long encoderPosRight=0;
long newpositionRight;
long oldpositionRight = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long vel;

void setup()
{
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder, RISING);  // encoDER ON PIN 2
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
}

void loop()
{
newpositionRight = encoderPosRight;
newtime = millis();
vel = (newpositionRight-oldpositionRight) * 10000 /(newtime-oldtime);  // 1000(to conver milli sec to sec)*60(to convert sec to minutes)/6(total number of pulses per rotation)=10000 (pulses per milli sec to rpm)
Serial.print ("speed of Right Motor = ");
Serial.println (vel);
oldpositionRight = newpositionRight;
oldtime = newtime;
delay(1000);
}

void doEncoder()
{
    encoderPosRight++;
    //Serial.println (encoderPos);
}


void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 encoderPosRight = 0;
 while(1)
 {
  if(encoderPosRight > ReqdShaftCountInt)
  {
    break;
  }
 } 
}

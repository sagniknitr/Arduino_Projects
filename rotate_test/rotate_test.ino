#define pwmA 11
#define poA 10
#define ptA 9
#define ptB 8
#define poB 7
#define pwmB 6


int speed_left=170, speed_right=165;

void setup() {
  // put your setup code here, to run once:

//pinMode(LED, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(poA, OUTPUT);
  pinMode(ptA, OUTPUT);
  pinMode(ptB, OUTPUT);
  pinMode(poB, OUTPUT);
  pinMode(pwmB, OUTPUT);
  Serial.begin(9600);

}

void moveRight(void)
{
  //digitalWrite(pwmA, baseSpeed);
  //digitalWrite(pwmB, baseSpeed);
  speedControl();
  digitalWrite(poA, HIGH);
  digitalWrite(ptA, LOW);
  digitalWrite(poB, HIGH);
  digitalWrite(ptB, LOW);
}

void stopAll()
{
  digitalWrite(pwmA, 0);
  digitalWrite(pwmB, 0);

  digitalWrite(poA, 0);
  digitalWrite(ptA, 0);
  digitalWrite(poB, 0);
  digitalWrite(ptB, 0);
}


void speedControl()
{
  if((speed_left>=255)&&(speed_right>=255))
  {
    analogWrite(pwmA, 254);
    analogWrite(pwmB, 254);
  }
  else if((speed_left>=255)&&(speed_right<255))
  {
    analogWrite(pwmA, 254);
    analogWrite(pwmB, speed_left);
    
  }
  else if((speed_left<255)&&(speed_right>=255))
  {
    analogWrite(pwmA, speed_left);
    analogWrite(pwmB, 254);
  }
  else
  {
    
    analogWrite(pwmA, speed_left);
    analogWrite(pwmB, speed_right);
    
  }
  }


void loop() {
  // put your main code here, to run repeatedly:


 speed_left=100, speed_right=90;
moveRight();

long T= 700.0;
Serial.println("ergerger");

delay(T);
stopAll();

delay(200);
}

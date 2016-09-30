#define pwmA 11
#define poA 10
#define ptA 9
#define ptB 8
#define poB 7
#define pwmB 6
#define array1 31
#define array2 33
#define array3 35
#define array4 37
#define array5 39
#define array6 41
#define array7 43
#define array8 45
#define InterruptPin 2
#define RECV_PIN 3
#define LED 13
#define baseSpeed 170
#define POILED 12
#define RECV_PIN 3
#define LED 13
#define baseSpeed 170
#define POILED 12
#define InterruptPin 2


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "poi.h"
#include <IRremote.h>


/*void moveStraight(void);
void moveLeft(void);
void moveRight(void);
void moveBack(void);
void stopAll(void);*/

/****************ALL GLOBAL VARIABLES***********************/

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float heading=0.0;
float final_heading=0.0;
float threshold=8;
float rotatethreshold=20.0;
float error=0.0;
float kp=0.2,kd=0.0,ki=0;
int speed_left, speed_right;
float error_mag=0.0;
float error_diff=0.0;
float prev_error=0.0;
float offset_right_RT = 90;
float offset_left_RT = 1000;
float offset_left_LT = 80;
float offset_right_LT = 80;
float offset_left = 100;
float offset_right = 90;
float threshold_POI=12.0;
float error_POI=0.0;
float error_diff_POI=0.0;
float prev_error_POI=0.0;
float error_mag_POI=0.0;
float poiangle=0.0;


IRrecv irrecv(RECV_PIN);
/************************************************************?
/*void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delayMicroseconds(50);
}*/

volatile char poi_interrupt=0;

void setup()
{
  //delay(2000);
  pinMode(LED, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(poA, OUTPUT);
  pinMode(ptA, OUTPUT);
  pinMode(ptB, OUTPUT);
  pinMode(poB, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(POILED,OUTPUT);
//  pinMode(pin, OUTPUT);
  pinMode(array1,INPUT);
  pinMode(array2,INPUT);
  pinMode(array3,INPUT);
  pinMode(array4,INPUT);
  pinMode(array5,INPUT);
  pinMode(array6,INPUT);
  pinMode(array7,INPUT);
  pinMode(array8,INPUT);
   
 // Serial.begin(9600);

 
  
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
    irrecv.enableIRIn(); // Start the receiver
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);

  /*for(int p = 0; p <= 4; p++)
  {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }*/

}
 

}


void handlePOIinterrupt()
{
 
  Serial.println("inside handle interrupt functiom");
  final_heading=roundOne(IRread);
  if(final_heading<0)
  {
    // STOP BOT HERE

    // WAIT INDEFINITELY
       while(1);
  }
  //delay(100);
   Serial.println("HEADING------------> " + String(final_heading));

   Serial.println("Now bot will rotate to orient towards the POI centre");
      
      heading=getHeading();
      Serial.println("Final Heading");
      Serial.println(final_heading);
      //while(1){;}
      align_forward();
      stopAll();
      error=final_heading-heading;
      Serial.print("Initial heading");
      Serial.println(error);
      while(abs(error) >rotatethreshold)
      {
        
         Serial.println("Now entering rotate mode");
      heading=getHeading();
      if(heading<360)
      {
      error=final_heading-heading;
      error_diff=error-prev_error;
      error_mag=abs(error);
      
      if(error_mag < 180 )
      {
      if(error>=threshold)
      {
        speed_right=abs(offset_right_LT+kp*error+kd*error_diff);
        speed_left=abs(offset_left_LT-kp*error+kd*error_diff);
        moveLeft();
                
      }
      else if(error<(-threshold))
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveRight();
      }
      else if((error<threshold)&&(error>(-threshold)))
      {
        
       /* speed_right=140;           //these speed are chosen arbitarily
        speed_left=150;
        moveStraight();*/
        stopAll();
      }
        
      }
      
      else if(error_mag > 180)
      {  if(error>=threshold)
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveRight();
                
      }
      else if(error<(-threshold))
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveLeft();
      }
      else if((error<threshold)&&(error>(-threshold)))
      {
        
        /*speed_right=140;           //these speed are chosen arbitarily
        speed_left=150;
        moveStraight();*/
        stopAll();
        break;
      }
        

        
      }
      
      else
      {
        Serial.println("IILEGALVLAUE");
        Serial.println(error_mag);
        
        stopAll();
        
      }

      prev_error=error;
      Serial.print("SpeedRIGHT");
      Serial.println(speed_right);
      Serial.print("SpeedLEFT");
      Serial.println(speed_left);
      Serial.print("Error");
      Serial.println(error);
      }
      else
      {
        Serial.println("Wrong serial vvalue");
      }


        
      }

      Serial.println("bot has rotated succesfully");
      //align_forward();
      stopAll();

   
}

char start_node_flag =0;

void align_forward()
{
  speed_right=90;
  speed_left=100;
  moveStraight();
  delay(900);
}

void forward_short()
{
  
}




void loop() 
{
  if(start_node_flag==0)
  {
    final_heading = roundOne(IRread);
    Serial.println("First node passed");
    start_node_flag=1;

    // ALIgn(move just over the current poi)  and rotate 2 next heading and move straight for around 1s

    align_forward();//only short forward
    stopAll();
    /******************************rotate to next heading****************************************************************************/
                Serial.println("Now bot will rotate to orient towards the POI centre");
      
      heading=getHeading();
      error=final_heading-heading;
      Serial.print("Initial heading");
      Serial.println(error);
      while(error >rotatethreshold)
      {
        
         Serial.println("Now entering rotate mode");
      heading=getHeading();
      if(heading<360)
      {
      error=final_heading-heading;
      error_diff=error-prev_error;
      error_mag=abs(error);
      
      if(error_mag < 180 )
      {
      if(error>=threshold)
      {
        speed_right=abs(offset_right_LT+kp*error+kd*error_diff);
        speed_left=abs(offset_left_LT-kp*error+kd*error_diff);
        moveLeft();
                
      }
      else if(error<(-threshold))
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveRight();
      }
      else if((error<threshold)&&(error>(-threshold)))
      {
        
       /* speed_right=140;           //these speed are chosen arbitarily
        speed_left=150;
        moveStraight();*/
        stopAll();
      }
        
      }
      
      else if(error_mag > 180)
      {  if(error>=threshold)
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveRight();
                
      }
      else if(error<(-threshold))
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveLeft();
      }
      else if((error<threshold)&&(error>(-threshold)))
      {
        
        /*speed_right=140;           //these speed are chosen arbitarily
        speed_left=150;
        moveStraight();*/
        stopAll();
        break;
      }
        

        
      }
      
      else
      {
        Serial.println("IILEGALVLAUE");
        Serial.println(error_mag);
        
        stopAll();
        
      }

      prev_error=error;
      Serial.print("SpeedRIGHT");
      Serial.println(speed_right);
      Serial.print("SpeedLEFT");
      Serial.println(speed_left);
      Serial.print("Error");
      Serial.println(error);
      }
      else
      {
        Serial.println("Wrong serial vvalue");
      }


        
      }

      Serial.println("bot has rotated succesfully");
      align_forward();
      stopAll();

    /******************************************************************************************************************************/
    
  }

  
  
 
    //---->
    Serial.println("-----> Attaching Interrupt....");

        
    attachInterrupt(digitalPinToInterrupt(InterruptPin), blink, RISING);
    //delay(1000);
    // ATTATCH INTERRUPT AFTER TURNING TO HEADING AND MOVING SOME DISTANCE...
  
    // wait for data and then go for the following functions as needed //
      // moveStraight(void)  ======  moves Straight
      // moveLeft(void) ============ moves LEFT
      // moveRight(void) =============== moves RIGHT
      // moveBack(void) ================= moves Back // turns About // set Data in function before Upload
      // stopAll(void) ==================== stops all motors //

    /*  moveStraight();
      delay(10000);
      stopAll();
      delay(1300);*/

     //first check initial heading


     //then align the bot according to heading.the bot should rotate until it reached some error


     //then move bot until black is detected


     //if possible align the bot to heading while moving

     //on

     Serial.println("POI interruptvalue");
     Serial.println(poi_interrupt);

     if(!poi_interrupt)
     {
      heading=getHeading();
      error=final_heading-heading;
      Serial.print("Initial heading");
      Serial.println(error);

      if(abs(error) > rotatethreshold)
      {
        Serial.println("Now entering rotate mode");
      heading=getHeading();
      if(heading<360)
      {
      error=final_heading-heading;
      error_diff=error-prev_error;
      error_mag=abs(error);
      
      if(error_mag < 180 )
      {
      if(error>=threshold)
      {
        speed_right=abs(offset_right_LT+kp*error+kd*error_diff);
        speed_left=abs(offset_left_LT-kp*error+kd*error_diff);
        moveLeft();
                
      }
      else if(error<(-threshold))
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveRight();
      }
      else if((error<threshold)&&(error>(-threshold)))
      {
        
       /* speed_right=140;           //these speed are chosen arbitarily
        speed_left=150;
        moveStraight();*/
        stopAll();
      }
        
      }
      
      else if(error_mag > 180)
      {  if(error>=threshold)
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveRight();
                
      }
      else if(error<(-threshold))
      {
        speed_right=abs(offset_right-(kp*error+kd*error_diff));
        speed_left=abs(offset_left+kp*error+kd*error_diff);
        moveLeft();
      }
      else if((error<threshold)&&(error>(-threshold)))
      {
        
        /*speed_right=140;           //these speed are chosen arbitarily
        speed_left=150;
        moveStraight();*/
        stopAll();
      }
        

        
      }
      
      else
      {
        Serial.println("IILEGALVLAUE");
        Serial.println(error_mag);
        
        stopAll();
        
      }

      prev_error=error;
      Serial.print("SpeedRIGHT");
      Serial.println(speed_right);
      Serial.print("SpeedLEFT");
      Serial.println(speed_left);
      Serial.print("Error");
      Serial.println(error);
      }
      else
      {
        Serial.println("Wrong serial vvalue");
      }
      }
      else                    // if bot is not rotating
      {

        Serial.println("Now entering movement mode");

     //if()
     //if(!poi_interrupt) 
    //{
     // Serial.println("Inside interrupt mode");
      
      heading=getHeading();
      error_POI=final_heading-heading;
      error_diff_POI=error_POI-prev_error_POI;
      error_mag_POI=abs(error_POI);
      
     
      if(error_POI>=threshold_POI)
      {
        speed_right=abs(offset_right+kp*error_POI+kd*error_diff_POI);
        speed_left=abs(offset_left-kp*error_POI+kd*error_diff_POI);
        moveStraight();
                
      }
      else if(error_POI<(-threshold_POI))
      {
        speed_right=abs(offset_right-(kp*error_POI+kd*error_diff_POI));
        speed_left=abs(offset_left+kp*error_POI+kd*error_diff_POI);
        moveStraight();
      }
      else if((error_POI<threshold_POI)&&(error_POI>(-threshold_POI)))
      {
        
        speed_right=80;           //these speed are chosen arbitarily
        speed_left=90;
        moveStraight();
        //stopAll();
      }
      Serial.print("Speed right in movement mode");
      Serial.println(speed_right);
      Serial.print("Speed left in movement mode");
      Serial.println(speed_left);
    //}
     /* else
        {
         //wait for some time to take sensor values

         //if possible move the bot forward for minimum distance
         Serial.println("Now take IR LED values");
         handlePOIinterrupt();
         poi_interrupt = 0;
   
        }*/
      prev_error_POI=error_POI;
     }
      
     }

     else
     {
        //wait for some time to take sensor values

         //if possible move the bot forward for minimum distance
         stopAll();
         
         Serial.println("--->Disabled Interrupt");
         Serial.println(poiangle);
         //while(1);
         Serial.println("Now in interrupt Mode");
         
         Serial.println("Now take IR LED values");
         //delay(200);
         Serial.println("angle to be rotated");

         
                    
         Serial.println("now bot will take heading");
         handlePOIinterrupt();
         //delay(200);
         poi_interrupt = 0;
     }
      
     }

     


void moveStraight(void)
{
 // digitalWrite(pwmA, baseSpeed);
  //digitalWrite(pwmB, baseSpeed);
  speedControl();
  digitalWrite(poA, HIGH);
  digitalWrite(ptA, LOW);
  digitalWrite(poB, HIGH);
  digitalWrite(ptB, LOW);
}

void moveRight(void)
{
  //digitalWrite(pwmA, baseSpeed);
  //digitalWrite(pwmB, baseSpeed);
  speedControl();
  digitalWrite(poA, LOW);
  digitalWrite(ptA, HIGH);
  digitalWrite(poB, HIGH);
  digitalWrite(ptB, LOW);
}

void moveLeft(void)
{
  //digitalWrite(pwmA, baseSpeed);
  //digitalWrite(pwmB, baseSpeed);
  speedControl();
  digitalWrite(poA, HIGH);
  digitalWrite(ptA, LOW);
  digitalWrite(poB, LOW);
  digitalWrite(ptB, HIGH);
}

void moveback(void)
{
  //digitalWrite(pwmA, baseSpeed);
  //digitalWrite(pwmB, baseSpeed);
  speedControl();
  digitalWrite(poA, LOW);
  digitalWrite(ptA, HIGH);
  digitalWrite(poB, LOW);
  digitalWrite(ptB, HIGH);
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

float getHeading(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle =0.2456;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  return headingDegrees;
  delayMicroseconds(500);                   //previous it was seconds
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

float POIAngle()
{
    char flag[8]={0,0,0,0,0,0,0,0};
    int val[8]={-4,-3,-2,-1,1,2,3,4};
    float total_angle=0.0;
    float counter=0;
    //state=LOW;
    //while(state==LOW);
    //while(1){
    if(digitalRead(array1)==HIGH)
    {flag[0]=1;
    //Serial.println("8");
    }
    if(digitalRead(array2)==HIGH)
    {flag[1]=1;
    //Serial.println("7");
    }
    //}
     if(digitalRead(array3)==HIGH)
    {flag[2]=1;
    //Serial.println("6");
    }
    //}
      if(digitalRead(array4)==HIGH)
    {flag[3]=1;
    //Serial.println("5");
    }
      if(digitalRead(array5)==HIGH)
    {flag[4]=1;
    //Serial.println("4");
    }
     if(digitalRead(array6)==HIGH)
    {flag[5]=1;
    //Serial.println("3");
    }

      if(digitalRead(array7)==HIGH)
    {flag[6]=1;
    //Serial.println("2");
    }

        if(digitalRead(array8)==HIGH)
    {flag[7]=1;
    //Serial.println("1");
    }
    
    for(int i=0;i<8;i++)
    {
      if(flag[i]==1)
      {
        total_angle+=val[i];
        counter+=1;
        
      }
    }
     //Serial.println("total angle");
    //Serial.println(total_angle);
    total_angle=total_angle/counter;
    //Serial.print("total angle");
    //Serial.println(total_angle);

    return total_angle;
    
    
    
    //delayMicroseconds(1000);
}
  
void blink() {
  detachInterrupt(digitalPinToInterrupt(InterruptPin));
   poi_interrupt = 1;
    poiangle=POIAngle();
    //Serial.println(poiangle);

}


int dec4NEC(unsigned long i)
{
   unsigned char MSB = (i>>24)&0xff;
   unsigned char LSB = (i>>8)&0xff;

   int o = MSB<<8 | LSB;
   return o;
}

int IRread()
{ 
 decode_results results;
 irrecv.resume(); //clear buffer
 while(!irrecv.decode(&results)); // wait unless not received
 int data =  dec4NEC(results.value);
 //irrecv.resume(); // Receive the next value
 //Serial.println("IR data received ---> " + String(data));
 return data; 
}

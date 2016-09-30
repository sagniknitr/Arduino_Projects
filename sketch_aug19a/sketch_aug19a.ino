/*
Self balancing Bot by Dipam Chakraborty and Sagnik Basu
Motors - 4,5,6,7 (PORTD PD7-PD4)
Motor PWM - 9,10
IMU used - MPU 6050/9150
IMU adress = 0x68
IMU interrupt - 2/IN0
LCD - R/S - 3 , E - 8 , Data - A0-A3
*/

#include <Wire.h>

#define mpwmr 9
#define mpwml 10
#define mleftb 42
#define mleftf 5
#define mrightb 6
#define mrightf 7
#define BluetoothBaud 57600
#define imu_interrupt 0
#define IMUAddress 0x68
#define I2C_TIMEOUT 1000

//LiquidCrystal lcd(3, 8, A3, A2, A1, A0);

//Controller variables
double currentInput, avgInput = 0.0;
int count = 0, dataCount = 0;
bool startOk = false,speedchanged = true;

int angularLimit = 12;
int oscillateSpeed = 110;
int oscillateDelay = 80;
int maxSpeed = 255, minSpeed = oscillateSpeed;
int targetPitch;


double prev_error = 0.0, sum_error = 0.0;
uint32_t prev_time;


//PID constants
double kp = 0.5;
double kd = 1000.0;
double ki = 0.001;
 

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter

uint32_t timer;
uint32_t timeStart;
uint8_t i2cData[14]; // Buffer for I2C data initial 14

int motorSpeed = 255;


double prevacc[3] = {0,0,0}; 

void setup()
{
	basics_Init();

	//lcd.begin(16, 2);
	//lcd.clear();
	//lcd.setCursor(0, 0);
	//lcd.print("Reset");
	//delay(1000);
	//lcd.clear();
	//lcd.print("Balance");

	//lcd.setCursor(0, 1);
	//lcd.print("Wait");
	Serial.print("Wait");

	imu_Init();

	//lcd.clear();
	//lcd.setCursor(0,0);
	//lcd.print("Ready");
	Serial.print("Ready");

	timeStart = millis();
}

void loop()
{
	while (1)
	{
                //Routine for remotely changing pid values
		/* Update all the values */
		while (i2cRead(0x3B, i2cData, 14));
		accX = ((i2cData[0] << 8) | i2cData[1]);
		accY = ((i2cData[2] << 8) | i2cData[3]);
		accZ = ((i2cData[4] << 8) | i2cData[5]);
                
                if(abs(prevacc[2]-accZ)>2500)
                {
         //         Serial.println(String(prevacc[2] - accZ) + " " + String(millis()/100));
                  //Serial.println("UP " + String(millis()/100));
                }
                /*else if(prevacc[2] - accZ<-2500)
                {
                  Serial.println("Down " + String(millis()/100));
                }*/
                prevacc[0] = accX;prevacc[1] = accY;prevacc[2] = accZ;
		tempRaw = (i2cData[6] << 8) | i2cData[7];
		gyroX = (i2cData[8] << 8) | i2cData[9];
		gyroY = (i2cData[10] << 8) | i2cData[11];
		gyroZ = (i2cData[12] << 8) | i2cData[13];

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		double roll = atan2(accY, accZ) * RAD_TO_DEG;
		double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

		double gyroXrate = gyroX / 131.0; // Convert to deg/s
		double gyroYrate = gyroY / 131.0; // Convert to deg/s

		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if (roll < -90 || roll > 90)
		{
			compAngleX = roll;
			gyroXangle = roll;
		}
		gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
		gyroYangle += gyroYrate * dt;

		compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

                if(compAngleY > 50)
                {
                  Serial.println("Front");
                }
                else if(compAngleY < -50)
                {
                  Serial.println("Back");
                }
                else if(compAngleX > 50)
                {
                  Serial.println("Right");
                }
                else if(compAngleX < -50)
                {
                  Serial.println("Left");
                }
                else
                {
                  Serial.println("Standby");
                }
              //  Serial.println("AccX " + String(accX) +  "\tAccY " + String(accY) + "\tAccZ " + String(accZ));
		//Filtering complete
               // Serial.print("Pitch " + String(compAngleY) + "  ");
               // Serial.println("Roll " + String(compAngleX));
                delay(10);	}
}
void basics_Init()
{
	pinMode(mpwmr, OUTPUT);
	pinMode(mpwml, OUTPUT);
	pinMode(mleftb, OUTPUT);
	pinMode(mrightb, OUTPUT);
	pinMode(mleftf, OUTPUT);
	pinMode(mrightf, OUTPUT);

	Serial.begin(BluetoothBaud);
}
void imu_Init()
{
if(ARDUINO >= 157)
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
else
	TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz

	i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	i2cData[2] = 0x00; // Set Gyro Full Scale Range to �250deg/s
	i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to �2g
	while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
	while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

	while (i2cRead(0x75, i2cData, 1));
	if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
		Serial.print("Error reading sensor");
		while (1);
	}
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) 
{
		return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	Wire.write(data, length);
	uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
	if (rcode) {
		Serial.print(F("i2cWrite failed: "));
		Serial.println(rcode);
	}
	return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) 
{
	uint32_t timeOutTimer;
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
	if (rcode) 
	{
		Serial.print(F("i2cRead failed: "));
		Serial.println(rcode);
		return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
	}	
	Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
	for (uint8_t i = 0; i < nbytes; i++) 
	{
		if (Wire.available())
			data[i] = Wire.read();
		else 
		{
			timeOutTimer = micros();
			while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
			if (Wire.available())
				data[i] = Wire.read();
			else 
			{
				Serial.println(F("i2cRead timeout"));
				return 5; // This error value is not already taken by endTransmission
			}
		}
	}
	return 0; // Success
}




#include <Kalman.h>
#include<Servo.h>
#include<Wire.h>
#include<I2Cdev.h>
#include<MPU6050.h>
MPU6050 CBgoc;
Kalman kalmanX;
//IMU 6050=====
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;
float accXangle;
float gyroXangel;
float kalAngelX;

unsigned long timer;
uint8_t i2cData[14];
float CurrentAngle;
// MOTOR
int AIN1 = 4;
int AIN2 = 5;
int BIN1 = 6;
int BIN2 = 7;
int CIN1 = 9;
int CIN2 = 10;
int speed;
// PID
const float Kp = 20;//5.5
const float Ki = 1;//1.5
const float Kd = 6;//2
float pTerm, iTerm, dTerm, integrated_error, last_error, error;
const float K = 1.9*1.12;
#define   GUARD_GAIN   10.0
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))

void setup() 
{
  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz 
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling 
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while(i2cWrite(0x19,i2cData,4,false)); 
  while(i2cWrite(0x6B,0x01,true));
  while(i2cRead(0x75,i2cData,1));
  if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
  Serial.print(F("Error reading sensor"));
  while(1);
  }
  delay(100); 
//Kalman=========
  while(i2cRead(0x3B,i2cData,6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  kalmanX.setAngle(accXangle); 
  gyroXangel = accXangle; 
  timer = micros();
  
}
  
void loop()
{
  Serial.println(accX);
 // Serial.println(accY);
 // Serial.println(accZ);
  Serial.println(accXangle);
  Serial.println(CurrentAngle);
  runEvery(25)
  {
    dof();
    if(CurrentAngle <=181 && CurrentAngle >=179)
    {
      stop();
    }
    else
    {
      if(CurrentAngle < 230 && CurrentAngle >130)
      {
        Pid();
        Motors();
      }
      else
      {
        stop();
      }
    }
  } 
}
void Motors()
{
  if(speed > 0)
  {
    analogWrite(CIN1, speed);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(CIN2, speed);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
   }
   else
   {
    speed = map(speed,0,-255,0,255);
    analogWrite(CIN1, speed);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(CIN2, speed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
   }
}
void stop()
{
    speed = map(speed,0,-255,0,255);// -150, 150
    analogWrite(CIN1, speed);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(CIN2, speed);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
//  analogWrite(AIN1, 0);
//  analogWrite(AIN2, 0);
//  analogWrite(BIN1, 0);
//  analogWrite(BIN2, 0);
}
void Pid()
{
  error = 180 - CurrentAngle;  // 180 = level
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki*constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd*(error - last_error);
  //iTerm = Ki * (error + last_error);
  //dTerm = Kd * (error - last_error);
  last_error = error;
  speed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}
void dof()
{
  while(i2cRead(0x3B,i2cData,14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
   gyroY = ((i2cData[10] << 8) | i2cData[11]);
   gyroZ = ((i2cData[12] << 8) | i2cData[13]);
   accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
   double gyroXrate = (double)gyroX/131.0;
   CurrentAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);
   timer = micros();
}




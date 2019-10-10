#include <Wire.h>
#include <Servo.h>
#include "i2c.h"
#include "i2c_BMP280.h"
#include <SD.h>
#include <SPI.h>

// for tvc take reference coordinate axis as y
// tanx = a2/a1 , x = tan-1(a2/a1)
// do 90-x

/*
 * SCL - A5
 * SDA - A4
 * MOSI - 11
 * MISO -12 
 * SCK - 13
 * MPU6050 INT - 2 
*/

File DataFile;

const int mpu_addr = 0x68;

const int cs_pin = 10;

BMP280 bmp280;

float ax , ay , az , temp , gx , gy , gz;
float pressure , altitude , altitude_base;

#define SERVO_TVC_X 6
#define SERVO_TVC_Y 5
#define Start_button 9
#define LED_RED 7
#define LED_GREEN 8

#ifndef servo_x
  #define servo_x 1
#endif

#ifndef servo_y
  #define servo_y 0
#endif

bool START_STATE = false;

int angle_x , angle_y , prev_x , prev_y;

int button_state;

Servo Servo_tvc_x;
Servo Servo_tvc_y;


float GetAngle(float a , int servo_);
float mod(float x);
void ChangeServoAngle(int prev_angle , int current_angle , Servo servo);
bool CheckAltitudeChange(struct altitude alt);

struct Altitude
{
  float prev_altitude;
  float current_altitude;
  float base_altitude;
};

void setup() {

  Serial.begin(9600);

  pinMode(Start_button , INPUT);
  pinMode(LED_RED , OUTPUT);
  pinMode(LED_GREEN , OUTPUT);
  pinMode(cs_pin , OUTPUT);

  button_state = digitalRead(Start_button);

  if(button_state == HIGH)
  {
    START_STATE = true;
  }

  while(START_STATE!=true)
  {
    button_state = digitalRead(Start_button);

    if(button_state == HIGH)
    {
    START_STATE = true;
    }
  }

  Serial.println("initializing flight controller...");

  digitalWrite(LED_RED , HIGH);
  digitalWrite(LED_GREEN, LOW);
  delay(1000);
  digitalWrite(LED_RED , LOW);
  digitalWrite(LED_GREEN , HIGH);

  Wire.begin();
  Wire.beginTransmission(mpu_addr);
  if(Wire.write(0x6B)!= 0)
  {
    Serial.println("IMU (MPU6050) has been initialized");
  }
  else
  {
    Serial.println("problem with IMU");
    while(true)
    {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED , HIGH);
      delay(500);
      digitalWrite(LED_RED , LOW);
    }
  }
  Wire.write(0);
  Wire.endTransmission(true);

   if (bmp280.initialize())
   {
    Serial.println("altimeter barometer (bmp280) found");
   }
   else
    {
        Serial.println("altimeter barometer (bmp280) not found");

         while(true)
         {
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(LED_RED , HIGH);
          delay(500);
          digitalWrite(LED_RED , LOW);
         }      
    }

    Serial.println("initializing SD card");

    if(SD.begin())
    {
       Serial.println("SD card ready");
    }
    else 
    {
      Serial.println("SD card failed"); 

      while(true)
         {
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(LED_RED , HIGH);
          delay(500);
          digitalWrite(LED_RED , LOW);
         }      
    }

    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();

   Serial.println("testing servos");


   Servo_tvc_x.attach(SERVO_TVC_X);
   Servo_tvc_y.attach(SERVO_TVC_Y);

   Servo_tvc_x.write(90);
   Servo_tvc_y.write(90);
   
   for(int i = 0;  i<= 180; i++)
   {
    Servo_tvc_x.write(i);
    Servo_tvc_y.write(i);

    //delay(25);
    
   }

   for(int i = 180;  i>=0; i--)
   {
    Servo_tvc_x.write(i);
    Servo_tvc_y.write(i);

    //delay(25);
    
   }

   Servo_tvc_x.write(90);
   Servo_tvc_y.write(90); 

   Serial.println("all sensors functioning testing done...");

   bmp280.awaitMeasurement();

    float temperature;
    bmp280.getTemperature(temperature);

    float pressure;
    bmp280.getPressure(pressure);

   bmp280.getAltitude(altitude_base);

   bmp280.triggerMeasurement();
   
   Serial.println("base altitude is : ");
   Serial.print(altitude_base);

   prev_x = 90;
   prev_y = 90;
}

void loop() {

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr,14,true);
  ax=Wire.read()<<8|Wire.read();
  ay=Wire.read()<<8|Wire.read();
  az=Wire.read()<<8|Wire.read();
  float Temp=Wire.read()<<8|Wire.read();
  gx=Wire.read()<<8|Wire.read();
  gy=Wire.read()<<8|Wire.read();
  gz=Wire.read()<<8|Wire.read();

  bmp280.awaitMeasurement();

  bmp280.getTemperature(Temp);
  
  float pressure;
  bmp280.getPressure(pressure);

  static float alt;
  bmp280.getAltitude(alt);

  alt = alt - altitude_base;

  // code to debug whether sensors are reporting accurate data
  /*Serial.println("ax :");
  Serial.print(ax);
  Serial.print("ay :");
  Serial.print(ay);
  Serial.print("az :");
  Serial.print(az);
  Serial.print("altitude_base:");
  Serial.print(altitude_base);
  Serial.print("altitude:");
  Serial.print(alt);
  Serial.println();

  delay(500);*/

  prev_x = TVS(ax , Servo_tvc_x , servo_x , prev_x);

  if(prev_x == -1)
  {
    digitalWrite(LED_RED , HIGH);
    digitalWrite(LED_GREEN , LOW);
  }

  prev_y = TVS(ay , Servo_tvc_y , servo_y , prev_y);

  if(prev_y == -1)
  {
    digitalWrite(LED_RED , HIGH);
    digitalWrite(LED_GREEN , LOW);
  }
  
  LogData(ax , ay , az , alt , pressure , Temp);
}

int TVS(float a , Servo servo , int servo_ , int prev_angle)
{
   float angle = GetAngle(a , servo_);

   if(angle == -1)
   {
      return -1;
   }

   if(angle != prev_angle)
   {
    ChangeServoAngle(prev_angle ,angle , servo);
     return angle;
   }
   else
   {
    return prev_angle;
   }
}

void ChangeServoAngle(int prev_angle , int current_angle , Servo servo)
{
   if(prev_angle != current_angle)
   {
     if(prev_angle>current_angle)
     {
       for(int i = prev_angle; i>=current_angle; i--)
       {
         servo.write(i);
         delay(10);
       }
     }
     else
     {
      for(int i = prev_angle; i<= current_angle; i++)
      {
        servo.write(i);
        delay(10);
      }
     }
   }
   else
   {
    servo.write(prev_angle);
   }
}

float mod(float x)
{
  if(x<0)
  {
    x = x*-1;
  }
  else
  {
    x = x;
  }

  return x;
}

float GetAngle(float a , int servo_)
{
  float angle;
  
  if(servo_ != 0)
  {
    if(servo_ == 1)
    {
       angle = map(a , -17000,17000 , 179 , 0);
    }
    else
    {
      angle = map(a , 17000 , -17000 , 179 , 0);
    }
  }
  else
  {
    angle = -1;
  }

  return angle;
  
}

/*float GetAngle(float a2 , float a1,int servo_)
{
  float angle_dir = atan(a2/a1);

  angle_dir = (angle_dir/PI)*180;

  float angle_corrected = angle_dir - 90;

  /*if(servo_ == 1)
  {
    angle_corrected = map(angle_corrected , -90 , 0 , 90 , 180);
  }
  else
  {
    
    angle_corrected = map(angle_corrected,-90 , 90 , 0 , 180);
  }
    
  
  return angle_corrected;
}*/



//RETURN ALTITUDE
float GetAltitude()
{
  
}

//START FLIGHT FUNCTION

bool Start()
{
  
}

//CHECK THRUST AND IF +VE DEPLOY PARACHUTE
void CheckThrust(float az)
{
  if(az>0)
  {
    DeployParachute();
  }
  
}

//DEPLOY PARACHUTE FUNCTION
void DeployParachute()
{
  
}

//WRITE DATA TO MICRO SD CARD
void LogData(float ax ,float ay ,float az ,float alt ,float temp ,float pressure )
{
    DataFile = SD.open("Data.txt",FILE_WRITE);

    if(DataFile)
    {
      DataFile.println(ax);
      DataFile.print(",");
      DataFile.print(ay);
      DataFile.print(",");
      DataFile.print(az);
      DataFile.print(",");
      DataFile.print(alt);
      DataFile.print(",");
      DataFile.print(temp);
      DataFile.print(",");
      DataFile.print(pressure);
      DataFile.println();

      DataFile.close();
    }
    else
    {
      Serial.println("ERROR OPENING FILE");

    while(true)
    {
      
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED , HIGH);
      delay(500);
      digitalWrite(LED_RED , LOW);
      
    }
    
    }
      
}

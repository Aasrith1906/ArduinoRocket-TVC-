
/* The following is the 2nd version of the arduino flight controller
 *  this code has been written by Aasrith Chennapragada
 *  first it tests whether all the hardware is working or not
 *  if all the tests are passed , the rocket checks for a change in the altitude , signifiying a launch
 *  after launch has been detected , the rocket will start its thrust vector control 
 *  it will do so until until the y axis acceleration is negative and the altitude is decreasing
 *  after it is detected that the rocket is falling it will deploy the parachute
 *  
 *  All the flight data information like acceleration , altitude , pressure inside , temperature , is recorded and stored on an
    SD card
 */



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


BMP280 bmp280;

float ax , ay , az , temp , gx , gy , gz;
float pressure , altitude , altitude_base;

#ifndef SERVO_TVC_X 
  #define SERVO_TVC_X 8
#endif

#ifndef SERVO_TVC_Z
  #define SERVO_TVC_Z 9
#endif

#ifndef Start_button
  #define Start_button 10
#endif

#ifndef LED_RED
  #define LED_RED 7
#endif

#ifndef LED_GREEN 
  #define LED_GREEN 6
#endif

#ifndef PARACHUTE_RELAY_PIN
  #define PARACHUTE_RELAY_PIN 5
#endif

#ifndef CS_PIN 
  #define CS_PIN 4
#endif

/*#ifndef servo_x
  #define servo_x 1
#endif

#ifndef servo_z
  #define servo_z 0
#endif*/


int servo_x = 1;

int servo_z = 0;

bool START_STATE = false;

int angle_x , angle_z , prev_x , prev_z;

int button_state;

Servo Servo_tvc_x;
Servo Servo_tvc_z;


/*float GetAngle(float a , int servo_);
float mod(float x);
void ChangeServoAngle(int prev_angle , int current_angle , Servo servo);
bool CheckAltitudeChange(struct altitude alt);*/


int TVS(float a , Servo servo , int servo_ , int prev_angle , float base_altitude);

void ChangeServoAngle(int prev_angle , int current_angle , Servo servo);

float GetAngle(float a , int servo_);

float GetAltitude(float base_altitude);

bool Start(float base_altitude,float altitude);

void DeployParachute(float base_altitude);

bool CheckAngle(int angle);

void LogData(float ax ,float ay ,float az ,float alt ,float temp ,float pressure );

struct altitude
{
  float prev_alt;
  float current_alt;
  float base_altitude;
};

struct altitude alt_change;

void setup() {

  Serial.begin(9600);

  

  pinMode(Start_button , INPUT);
  pinMode(LED_RED , OUTPUT);
  pinMode(LED_GREEN , OUTPUT);
  pinMode(CS_PIN , OUTPUT);
  pinMode(PARACHUTE_RELAY_PIN , OUTPUT);


  //Waiting for a button input to start the flight controller

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

  //initializing testing 

  
  Serial.println("initializing flight controller...");

  // testing LED'S

  digitalWrite(LED_RED , HIGH);
  digitalWrite(LED_GREEN, LOW);
  delay(1000);
  digitalWrite(LED_RED , LOW);
  digitalWrite(LED_GREEN , HIGH);

  //Initializing MPU6050 , testing

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

  //Initializing BMP280 (altimeter and barometer)

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


   //Initializing SD card
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

    //Testing servos

   Serial.println("testing servos");


   Servo_tvc_x.attach(SERVO_TVC_X);
   Servo_tvc_z.attach(SERVO_TVC_Z);

   Servo_tvc_x.write(90);
   Servo_tvc_z.write(90);
   
   for(int i = 0;  i<= 180; i++)
   {
    Servo_tvc_x.write(i);
    Servo_tvc_z.write(i);

    delay(10);
    
   }

   for(int i = 180;  i>=0; i--)
   {
    Servo_tvc_x.write(i);
    Servo_tvc_z.write(i);

    delay(10);
    
   }

   //Centering servos
   
   Servo_tvc_x.write(90);
   Servo_tvc_z.write(90); 

   prev_x = 90;
   prev_z = 90;

   Serial.println("all sensors functioning testing done...");

   // waiting for a change in altitude which signifies launch

   bmp280.awaitMeasurement();

    float Temp;
    bmp280.getTemperature(Temp);

    float pressure;
    bmp280.getPressure(pressure);

   bmp280.getAltitude(altitude_base);

   bmp280.triggerMeasurement();
   
   Serial.println("base altitude is : ");
   Serial.print(altitude_base);

   alt_change.prev_alt = 0;
   alt_change.current_alt = 0;
   alt_change.base_altitude = altitude_base;

   

  //float Temp;

  bmp280.awaitMeasurement();

  bmp280.getTemperature(Temp);
  
  //float pressure;
  bmp280.getPressure(pressure);

  static float alt;
  bmp280.getAltitude(alt);


  bool start_ = Start(alt , altitude_base);

  while(!start_)
  {
    
  //float Temp;

  bmp280.awaitMeasurement();

  bmp280.getTemperature(Temp);
  
  //float pressure;
  bmp280.getPressure(pressure);

  //static float alt;
  bmp280.getAltitude(alt);

  start_ = Start(alt , altitude_base);

  }

  Serial.println("Starting Flight Controls....");
  
}

void loop() {

  
  //if(start_)
  //{

    // Read all the data from the sensors
    
      float Temp;
    
      bmp280.awaitMeasurement();
    
      bmp280.getTemperature(Temp);
      
      float pressure;
      bmp280.getPressure(pressure);
    
      static float alt;
      bmp280.getAltitude(alt);
    
      float altitude = alt - altitude_base;


      
    
      Wire.beginTransmission(mpu_addr);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(mpu_addr,14,true);
      ax=Wire.read()<<8|Wire.read();
      ay=Wire.read()<<8|Wire.read();
      az=Wire.read()<<8|Wire.read();
      Temp=Wire.read()<<8|Wire.read();
      gx=Wire.read()<<8|Wire.read();
      gy=Wire.read()<<8|Wire.read();
      gz=Wire.read()<<8|Wire.read();
    
      
    
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

      // TVS on the x axis
    
      prev_x = TVS(ax , Servo_tvc_x , servo_x , prev_x , altitude_base);
    
      if(prev_x == -1)
      {
        digitalWrite(LED_RED , HIGH);
        digitalWrite(LED_GREEN , LOW);
      }


      //TVS on the z axis
      //prev_z = TVS(az , Servo_tvc_z , servo_z , prev_z);
      
      prev_z = TVS(az , Servo_tvc_z , 1 , prev_z , altitude_base);
    
      if(prev_z == -1)
      {
        digitalWrite(LED_RED , HIGH);
        digitalWrite(LED_GREEN , LOW);
      }

      // checking if apogee has been reached , if so release parachute
      alt_change.prev_alt = alt_change.current_alt;
      alt_change.current_alt = altitude;

      if(ApogeeReached(alt_change))
      {
        DeployParachute(alt_change.base_altitude);
      }

      // Logging data
      
      LogData(ax , ay , az , alt , pressure , Temp);

  //}

}


//Final TVS function returns the angle at which the servo is pointing


int TVS(float a , Servo servo , int servo_ , int prev_angle , float base_altitude)
{
   float angle = GetAngle(a , servo_);

   if(angle == -1)
   {
      return -1;
   }

   if(angle != prev_angle)
   {

    if(!CheckAngle(angle))
    {
      
     ChangeServoAngle(prev_angle ,angle , servo);
      
     return angle;
     
    }

    else
    {
      DeployParachute(base_altitude);
    }
     
    
   }
   else
   {
    return prev_angle;
   }
}

//Function to change th angle of a servo (in a very specific way)

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


//Gets the TVS angle

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
float GetAltitude(float base_altitude)
{
  
  bmp280.awaitMeasurement();

  float Temp;
  
  bmp280.getTemperature(Temp);
  
  float pressure;
  bmp280.getPressure(pressure);

  static float altitude;
  bmp280.getAltitude(altitude);

  altitude = altitude - base_altitude;

  return altitude;

}

//START FLIGHT FUNCTION

bool Start(float base_altitude,float altitude)
{
    if(altitude > base_altitude)
    {
      return true;
    }
    else
    {
      return false;
    }
    
} 

//CHECK THRUST(CHECK DIRECTION OF ROCKET) AND IF +VE DEPLOY PARACHUTE
void CheckThrust(float ay , struct altitude alt)
{
 
  
}

bool ApogeeReached(struct altitude alt)
{
  if(alt.current_alt<alt.prev_alt)
  {
    return true;
  }
  else
  {
   return false;
  }
}



//DEPLOY PARACHUTE FUNCTION
void DeployParachute(float base_altitude)
{
   digitalWrite(PARACHUTE_RELAY_PIN , HIGH);

   float alt = GetAltitude(base_altitude);

   while(alt>0)
   {
    digitalWrite(LED_RED , HIGH);
    digitalWrite(LED_GREEN , HIGH);

    delay(500);

    digitalWrite(LED_RED , LOW);
    digitalWrite(LED_GREEN , LOW);
   }

   digitalWrite(LED_RED , LOW);
   digitalWrite(LED_GREEN , HIGH);

   while(true)
   {
    
   }
}

/*//ABORT FUNCTION IF AN ERROR IS DETECTED MID FLIGHT

void Abort()
{

}*/


bool CheckAngle(int angle)
{
  if(angle>135 || angle<45)
  {
    return true;
  }

  else
  {
    return false;
  }
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

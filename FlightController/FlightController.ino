#include <Wire.h>
#include <Servo.h>
#include "i2c.h"
#include "i2c_BMP280.h"


// for tvc take reference coordinate axis as y
// tanx = a2/a1 , x = tan-1(a2/a1)
// do 90-x

const int mpu_addr = 0x68;

BMP280 bmp280;

float ax , ay , az , temp , gx , gy , gz;
float pressure , altitude , altitude_base;

#define SERVO_TVC_X 7
#define SERVO_TVC_Y 10
#define Start_button 9
#define LED_RED 12
#define LED_GREEN 11

bool START_STATE = false;

int angle_x , angle_y , prev_x , prev_y;

int button_state;

Servo Servo_tvc_x;
Servo Servo_tvc_y;


float GetAngle(float a1 , float a2);

void setup() {

  Serial.begin(9600);

  pinMode(Start_button , INPUT);
  pinMode(LED_RED , OUTPUT);
  pinMode(LED_GREEN , OUTPUT);

  button_state = digitalRead(Start_button);

  if(button_state == HIGH)
  {
    START_STATE = true;
  }

  while(!START_STATE)
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

    float pascal;
    bmp280.getPressure(pascal);

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

  ax = (ax/16384)*9.81;
  ay = (ay/16384)*9.81;
  az = (az/16384)*9.81;

  bmp280.awaitMeasurement();

  bmp280.getTemperature(Temp);
  
  float pascal;
  bmp280.getPressure(pascal);

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


  int angle_x  = GetAngle(az,ax,0);

  
  if(angle_x != prev_x)
  {
    prev_x = angle_x;

    Servo_tvc_x.write(angle_x);
  }

  Serial.println(angle_x);

  
  
}


float GetAngle(float a2 , float a1,int servo_)
{
  int angle_dir = (int)atan(a2/a1);

  angle_dir = (angle_dir/PI)*180;

  int angle_corrected = angle_dir;

  angle_corrected = map(angle_corrected,-90 , 90 , 0 , 180);

  return angle_corrected;
}

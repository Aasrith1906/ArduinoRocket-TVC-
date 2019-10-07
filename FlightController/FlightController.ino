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

    delay(25);
    
   }

   for(int i = 180;  i>=0; i--)
   {
    Servo_tvc_x.write(i);
    Servo_tvc_y.write(i);

    delay(25);
    
   }

   Servo_tvc_x.write(90);
   Servo_tvc_y.write(90); 

   Serial.println("all sensors functioning testing done...");

   bmp280.awaitMeasurement();

    float temperature;
    //bmp280.getTemperature(temperature);

    float pascal;
    bmp280.getPressure(pascal);

   bmp280.getAltitude(altitude_base);

   bmp280.triggerMeasurement();
   
   Serial.println("base altitude is : ");
   Serial.print(altitude_base);
}

void loop() {
  

}

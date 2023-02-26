#include <Wire.h>
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1013.25
#include "I2Cdev.h" // Must Install Libary 
#include "MPU6050.h" // Must Install Libary
#include "Servo.h"

#define TCAADDR 0x70

Adafruit_BMP085 bmp;



MPU6050 mpu; // Initalising Motion Processing Unit
int16_t ax, ay, az; //Acclerometer values
int16_t gx, gy, gz; // Gyroscope Values

// Servo Objects
Servo servo1;
Servo servo2;
Servo servo3;

// Variables to hold value of XY co-ordinates
int valX;
int valY;
int prevValX;
int prevValY;


float altitude = 0;
float temp = 0.0;
  
void setup() 
{

  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  servo1.attach(9);// Pitch 
  servo3.attach(11);//Roll 
  servo2.attach(10);// Roll

// Try this to make multitasking//
 

  if (!bmp.begin())
   {
	    
   }
}

// Tells which MUX switch to change to
void tcaselect(uint8_t i) 
{
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


  
void FC() 
{

  

    tcaselect(3); // 3rd mux pin to use BMP180
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(seaLevelPressure_hPa * 100));
    Serial.println(" meters");
    
    Serial.println();
    delay(100);

    altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);


    // If falling or rising ascend or descend to operational altitude
    while(altitude <20.0 || altitude >65.0)
    {

      if(altitude <20.0)
      {
        servo1.write(45);

      }
      else if (altitude >65)
      {
        servo1.write(45);

      }

    }


  tcaselect(2);
 // Lines below gets the motion data and maps in polar
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  valX = map(ax, -17000, 17000, 0, 179);
  valY = map(ay, -17000, 17000, 0, 179);
  


  //Die Sensor reading note Temp sensor
  //float temp = mpu.getTemperature();
  //temp = float(temp + 521)/340 + 35.0;

  //Serial.print(" Temp = ");
  //Serial.print(temp);
  //Serial.println(" *C");

  Serial.println("This is the roll value:");
  Serial.println(valX);
  Serial.println("This is the pitch value");
  Serial.println(valY);
  delay(500);  
 


  valX=valX; // Starting angle is at 90
  //Serial.println(valX);
  if (valX != prevValX) // If angle is not the same as before
  {
    if(80 < valX && valX < 100) // If angle is in this range stay at 90 degrees
    {
      servo1.write(90);
    }
    else
    {
      // Moves servo to that angle
      servo1.write(valX);
      prevValX = valX;

    }
  
  }


  // Line below gets the motion data and maps in polar
  // Y axis value being retrieved 
  // This is Roll
  valY=valY; // Starting angle is 90 degrees
  //Serial.println(valY);
  if (valY != prevValY)
  {

     if(80 < valY && valY < 100) // If angle is bewteen these values stay at 90 degrees
    {
      servo2.write(90);
      servo3.write(90);
    }
    else
    {
      //Move servo to the angle
      servo2.write(valY);
      servo3.write(valY);
      prevValY = valY;

    }
  
  }
  //delay(50);
}






  
void loop() 
{
    FC();
}












/* Flight Controller Code V2
  By Saboor Rashid and Qamar Ilyas
  Last Modified 19/01/2023

  Source Code from 
  https://www.instructables.com/MPU6050-Controlled-Servo-Arm/

  Code has 2 servo for roll and pitch
  MPU-6050 calculates angle using the Motion 
  Processing Unit (MPU) and angle is given to 
  servo where movement occurs

  Cannot mount servo in line must be using a arm

  Added code to make it less sensitive to movements
  Placed code in a dedicated function called FC

*/


#include "Wire.h"
#include "I2Cdev.h" // Must Install Libary 
#include "MPU6050.h" // Must Install Libary
#include "Servo.h"

MPU6050 mpu; // Initalising Motion Processing Unit
int16_t ax, ay, az; //Acclerometer values
int16_t gx, gy, gz; // Gyroscope Values

// Servo Objects
Servo servo1;
Servo servo2;

// Variables to hold value of XY co-ordinates
int valX;
int valY;
int prevValX;
int prevValY;


void setup()
{
//Intialising MPU-6050 and Servo Pins
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  servo1.attach(9);// Pitch 
  servo2.attach(10);// Roll
}



void FC() // Flight Controller Function
{

 // Lines below gets the motion data and maps in polar
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  valX = map(ax, -17000, 17000, 0, 179);
  valY = map(ay, -17000, 17000, 0, 179);
  

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
    }
    else
    {
      //Move servo to the angle
      servo2.write(valY);
      prevValY = valY;

    }
  
  }
  delay(50);
}


void loop()
{
 FC();
}




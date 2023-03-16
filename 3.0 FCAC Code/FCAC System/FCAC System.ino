/* FCAC Program
  By Saboor Rashid and Qamar Ilyas
  Last Modified 09/02/2023

  Source Code from 
  https://www.instructables.com/MPU6050-Controlled-Servo-Arm/

  Source Codes from
  https://makersportal.com/blog/2019/02/06/arduino-pitot-tube-wind-speed-theory-and-experiment

  https://docs.arduino.cc/tutorials/4-relays-shield/4-relay-shield-basics

  The code works as intended and has temperature readings as well
  ready for individual and full testing.

  For testing will need to tell which relays to operate
  

*/

#include "Wire.h"
#include "I2Cdev.h" // Must Install Libary 
#include "MPU6050.h" // Must Install Libary
#include "Servo.h"


// Defining Relay pins 
int relay_1 = 4;
int relay_2 = 7;
int relay_3 = 8;
int relay_4 = 12;


//Routine for calculating the velocity from 
//a pitot tube and MPXV7002DP pressure differential sensor

float V_0 = 5.0; // supply voltage to the pressure sensor
float rho = 1.204; // density of air 

// parameters for averaging and offset
int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;

/*-------------------------------------------------------*/

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




/*-----------------------------------------------------*/





void setup()
{
//Intialising MPU-6050 and Servo Pins
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  servo1.attach(9);// Pitch 
  servo3.attach(11);//Roll 
  servo2.attach(10);// Roll

   

  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);


  // This used to set up the pitot tube
  for (int ii=0;ii<offset_size;ii++)
  {
    offset += analogRead(A0)-(1023/2);
  }
  offset /= offset_size;
}


/*-------------------------------------------------------*/




  // Flight Controller Function //

void FC() 
{

 // Lines below gets the motion data and maps in polar
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  valX = map(ax, -17000, 17000, 0, 179);
  valY = map(ay, -17000, 17000, 0, 179);
  
  float temp = mpu.getTemperature();
  temp = float(temp + 521)/340 + 35.0;

  Serial.print(" Temp = ");
  Serial.print(temp);
  Serial.println(" *C");
  
  delay(500);  
 
  
  Serial.println();

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


/*------------------------------------*/

 //Anemomerty Control Function //

void ACircuit()
{
  float adc_avg = 0; float veloc = 0.0;
  
  // average a few ADC readings for stability
  for (int ii=0;ii<veloc_mean_size;ii++){
    adc_avg+= analogRead(A0)-offset;
  }
  adc_avg/=veloc_mean_size;
  
  // make sure if the ADC reads below 512, then we equate it to a negative velocity
  if (adc_avg>512-zero_span and adc_avg<512+zero_span){
  } else{
    if (adc_avg<512){
      veloc = -sqrt((-10000.0*((adc_avg/1023.0)-0.5))/rho);
    } else{
      veloc = sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
    }
  }
  Serial.println(veloc); // print velocity
  delay(100); // delay for stability


  if(veloc < 9 )
  {
    digitalWrite(relay_1, LOW);



  }


  
  if(veloc >= 9)
  {
    digitalWrite(relay_1, HIGH);
    //digitalWrite(relay_2, HIGH);
    //digitalWrite(relay_3, HIGH);
    //digitalWrite(relay_4, HIGH);

    //delay(1000);




  }
   if(veloc > 40)  
  {
    digitalWrite(relay_1, LOW);
    //digitalWrite(relay_2, LOW);
    //digitalWrite(relay_3, LOW);
    //digitalWrite(relay_4, LOW);

    //delay(1000);

  }

}





void loop()
{
 //FC();
 //delay(100);
 ACircuit();
}





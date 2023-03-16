/* FCAC Program
  By Saboor Rashid and Qamar Ilyas
  Last Modified 16/03/2023

  Source Code from 
  https://www.instructables.com/MPU6050-Controlled-Servo-Arm/

  https://makersportal.com/blog/2019/02/06/arduino-pitot-tube-wind-speed-theory-and-experiment

  https://docs.arduino.cc/tutorials/4-relays-shield/4-relay-shield-basics

  https://lastminuteengineers.com/bmp180-arduino-tutorial/

  https://makeabilitylab.github.io/physcomp/advancedio/smoothing-input.html

  https://dronebotworkshop.com/dc-volt-current/

  The Code use MPU-6050, BMP180, Pitot Tube, MPXV7002DP, Servos, Arduino Relay Sheild,
  PCA9548A MUX switch and NCA-219 Current and Voltage Sensor




  

*/

#include "Wire.h"
#include "I2Cdev.h" // Must Install Libary 
#include "MPU6050.h" // Must Install Libary
#include "Servo.h"
#include <Adafruit_BMP085.h> // Install Libary
#include <SoftwareSerial.h>
#include <Adafruit_INA219.h>

#define TCAADDR 0x70 // Address for the MUX Switch




/*-----------RELAY PINS----------------*/

// Defining Relay pins 
int relay_1 = 4;
int relay_2 = 7;
int relay_3 = 8;
int relay_4 = 12;


/*-----------------PITOT TUBE & MPXV7002DP VARIABLES------------------*/

//Routine for calculating the velocity from 
//a pitot tube and MPXV7002DP pressure differential sensor

float V_0 = 5.0; // supply voltage to the pressure sensor
float rho = 1.204; // density of air 

// parameters for averaging and offset
int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;


// Below is for smoothing the wind speed //
const int SMOOTHING_WINDOW_SIZE = 10;
float _samples[SMOOTHING_WINDOW_SIZE];
int _curReadIndex = 0;
float _sampleTotal = 0;
float _sampleAvg = 0;

float unfilter_veloc = 0.0;
float filter_veloc = 0.0;




/*----------------MPU-6050 PINS & VARIABLES-------------------------*/

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


float temp = 0.0;
float altitude = 0.0;



/*---------------BMP-180 VARIABLES-----------------*/

Adafruit_BMP085 bmp; // Defining object for BMP-180
float pressure = 0;

/*--------------BLUETOOTH PINS & VARIABLES--------------------*/

SoftwareSerial B(2,3); //Pin 2 Rx and Pin 3 Tx
unsigned long previousMillis = 0;
const long interval = 120000; // 2 minutes in milliseconds
const long bluetoothInterval = 20000; // 20 seconds in milliseconds
unsigned long currentMillis = millis();


/*-----------ELECTRIC VARIABLE-------*/

Adafruit_INA219 ina219; // Defining object for INA219
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;



void setup()
{
  /*------Setting Up MPU-6050 AND Servos------*/
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  servo1.attach(9);// Pitch 
  servo3.attach(11);//Roll 
  servo2.attach(10);// Roll



  /*-----Setting up Relays-----*/
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);


//---------------------------------------------------------//


  /*-----Setting up Pitot tube----*/
  for (int ii=0;ii<offset_size;ii++)
  {
    offset += analogRead(A0)-(1023/2);
  }
  offset /= offset_size;

  for (int i = 0; i< SMOOTHING_WINDOW_SIZE;i++)
  {
    _samples[i] = 0.0;
  }


//-------------------------------------------------------------//

  /*----Setting up BMP-180----*/
  if (!bmp.begin()) // This checks for connection
  {
	    
  }
    //Get the current pressue to get accurate altitude
    pressure = bmp.readPressure();


//------------------------------------------------------------//

  /*-----Setting up Bluetooth----*/
  B.begin(9600);
  
  while (!Serial) 
  {
      
  }
 
//----------------------------------------------------------//

 /*----Setting up INA219----*/
  uint32_t currentFrequency;
    
  Serial.println("Hello!");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range.
  if (! ina219.begin()) 
  {
    //Serial.println("Failed to find INA219 chip");
    
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
 
  Serial.println("Measuring voltage and current with INA219 ...");

}



/*-------------MUX Switch Code-----------------------------*/

/* This code allows the MPU-6050, BMP180, MPXV7002DP to only 
   use A4 (SCL) and A5 (SDL) using I2C protocol. Connect multiple
   breakout boards without fulling up all I2C ports on Arduino
*/


void tcaselect(uint8_t i) 
{
  if (i > 7) return; // Only 8 switches to choose //
 

 //Selecting the switch//
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}






  /*------FLIGHTH CONTROLLER FUNCTION------*/

void alt ()
{
  tcaselect(7); // SC7 AND SD7 used for SCL and SDA

  
    /*
      //For Testing Purposes    
      Serial.print("Temperature = ");
      Serial.print(bmp.readTemperature());
      Serial.println(" *C");
    
      Serial.print("Pressure = ");
      Serial.print(bmp.readPressure());
      Serial.println(" Pa");

      Serial.print("Pressure at sealevel (calculated) = ");
      Serial.print(bmp.readSealevelPressure());
      Serial.println(" Pa");
    
    
      Serial.print("Real altitude, ");
      Serial.println(bmp.readAltitude(pressure));
      Serial.println(" Meters");
    */
  
    // Temperature and Altitude storing variable globally
    temp = bmp.readTemperature();
    altitude = bmp.readAltitude(pressure);


  /*    
    //This code is for when AWT goes to low or too high  
    while(altitude >=65.0 || altitude <=20.0)
    {

    
      // If below above cerain altitude elevator moves adjust altitude
      if( altitude <20.0)
      {
        servo1.write(45);


      }
      else if (altitude >65)
      {
        servo1.write(135);      

      }
    }

*/


}




void levelflight() 
{

  tcaselect(2); // Switch 2 for MUX for SCL and SDC

 // Lines below gets the motion data and maps in polar
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 valX = map(ax, -17000, 17000, 0, 179);
 valY = map(ay, -17000, 17000, 0, 179);
  

  /*
    This code is to find temperature of MPU6050
    Good for Condition Monitoring
    float temp = mpu.getTemperature(); // Getting the temp of silicon die
    temp = float(temp + 521)/340 + 35.0;

    Serial.print(" Temp = ");
    Serial.print(temp);
    Serial.println(" *C");
  */
  
   
 
  // This is PITCH!!!! //
  valX=valX; // Starting angle is at 90
  //Serial.println(valX);
  if (valX != prevValX) // If angle is not the same as before
  {
    if(80 < valX && valX < 100) // If angle is bewteen these values stay at 90 degrees, otherwise servo keep moving
   {
      // Moves servo to that angle
      servo1.write(90);
      
      
    }
   else
   {
       servo1.write(valX);
       prevValX = valX;
       
       
   }
  
  }


  // Y axis value being retrieved 
  // This is ROLL!!!!//
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
  
}



/*-----------ANAMOMETRY & PROTECTION FUNTION--------------*/


void ACircuit()
{

  /* Below code get raw windspeed values */
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
  //Serial.println(veloc); // print velocity
  delay(10); // delay for stability

  /*-------------------RUNNING AVERAGE FILTER-------------------------*/

  /*Below code does running average of raw wind speed data*/

  //Storing Local variable to global variable
  unfilter_veloc = veloc; 

  // Subtract last reading sample
  _sampleTotal = _sampleTotal - _samples[_curReadIndex];

  //Stores velocity value
  _samples[_curReadIndex] = veloc;

  //Add the reading to the total
  _sampleTotal = _sampleTotal + _samples[_curReadIndex];

  //Move to next position in the array
  _curReadIndex =   _curReadIndex + 1;


  // Rest counter for array back to start
  if(_curReadIndex >= SMOOTHING_WINDOW_SIZE)
  {
    _curReadIndex = 0;    
  }

  //Averaging the sample and storing in local variable//
  _sampleAvg = _sampleTotal/SMOOTHING_WINDOW_SIZE;

  // Transfering to global variable
  filter_veloc = _sampleAvg;


  
  /*******************************************************/
   //Testing code in Excel streamer 
  /*
    //Testing Filter 
    Serial.print("Raw,");
    Serial.print(unfilter_veloc);
    Serial.print(",");
    Serial.print("Processed");
    Serial.print(",");
    Serial.println(filter_veloc);
  */
  /*****************************************************/


  // Tells when the relays to close or open
  if(filter_veloc < 2.0 )
  {
    digitalWrite(relay_1, HIGH); //4 pin
    digitalWrite(relay_2, LOW);  //7 pin
    digitalWrite(relay_3, LOW);  //8 pin
    digitalWrite(relay_4, LOW);  //12 pin

  }

  if(filter_veloc >= 6.5)
  {
    digitalWrite(relay_1, LOW);
    digitalWrite(relay_2, HIGH);
    digitalWrite(relay_3, HIGH);
    digitalWrite(relay_4, HIGH);

  }
   if(filter_veloc > 20.0)  
  {
    digitalWrite(relay_1, HIGH);
    digitalWrite(relay_2, LOW);
    digitalWrite(relay_3, LOW);
    digitalWrite(relay_4, LOW);

  }

}



void electric()
{

  tcaselect(4); // SC4 AND SC4 being used on MUX
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  

  /* Outputting to excel streamer
    Serial.print("Load Voltage,");
    Serial.print(loadvoltage);
    Serial.print(",");
    Serial.print("Current");
    Serial.print(",");
    Serial.println(current_mA);
  */

/* If voltage higher than 4.6V Open Relay
  if (loadvoltage > 4.6 )
  {

    digitalWrite(relay_3, LOW);

  }
  
  //Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  //Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  //Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  //Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  //Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  //Serial.println("");

*/


}




void loop()
{


    // Code works just not outputting
    if (currentMillis - previousMillis >= interval) 
    {
        previousMillis = currentMillis;
        B.println(filter_veloc);
        B.print(",");
        B.print(altitude);
        B.print(",");
        B.print(loadvoltage);
        B.print(";");
        delay(bluetoothInterval);
    }


    alt();
    levelflight();
    ACircuit();
    electric();
 
}

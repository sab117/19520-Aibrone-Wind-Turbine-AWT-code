/*
Anemometry Control Code V2
By Saboor Rashid and Qamar Ilyas 
Last Modified 23/01/2023

Source Codes from
https://makersportal.com/blog/2019/02/06/arduino-pitot-tube-wind-speed-theory-and-experiment

https://docs.arduino.cc/tutorials/4-relays-shield/4-relay-shield-basics


Code uses a Pitot Tube to measure wind speed of processed into wind speed value by 
microcontroller. Relays operate at specfic thresold (Threshold to be set)
Relays are used to protect circuit on board Airborne Wind Turbine.

Still to test !!

*/




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


// Below is for smoothing the wind speed //
const int SMOOTHING_WINDOW_SIZE = 10;
float _samples[SMOOTHING_WINDOW_SIZE];
int _curReadIndex = 0;
float _sampleTotal = 0;
float _sampleAvg = 0;

float unfilter_veloc = 0.0;
float filter_veloc = 0.0;


void setup() 
{
  // Defining Relay to be outputs
  Serial.begin(9600);

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
  delay(10); // delay for stability


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

  
  if(filter_veloc <9)
  {
    digitalWrite(relay_1, HIGH);
    //digitalWrite(relay_2, HIGH);
    //digitalWrite(relay_3, HIGH);
    //digitalWrite(relay_4, HIGH);

    delay(1000);




  }
  else if(filter_veloc > 15)  
  {
    digitalWrite(relay_1, LOW);
    //digitalWrite(relay_2, LOW);
    //digitalWrite(relay_3, LOW);
    //digitalWrite(relay_4, LOW);

    delay(1000);

  }

}



void loop() 
{
   ACircuit();
 

}
// **/ I2C SENSOR ADDRESS:
// 1. The first data byte is the Status Byte (8-bit) and the second to fourth bytes are the compensated pressure output (24-bit).

// 2. The default address for the MPR Series is 24 (0x18)
// */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* I2C PRESSURE READING:
1. To read out a compensated pressure reading, the Master generates a START condition and sends the Sensor address followed
by a read bit (1).

2. Transmit up to 4 bytes of data
  * 1 byte: Status Byte (8-bit)
  * 2-4 byte: pressure output (24-bit).

*/

#include <Arduino.h>
#include "Wire.h"
#include <Adafruit_CircuitPlayground.h>
//#include "arduinoFFT.h"
#include "Plotter.h"

#define Pressure 0x18
#define Output_Measurement 0xAA
#define exit 0x00
#define Pressure_Min 0
#define Pressure_Max 300
#define Output_Min 419430
#define Output_Max 3774873

Plotter screen;

uint8_t stage;
//This array of 5 readings is used for the 5-point averager
int len = 5;
float reading[5];
int indexOfArray = 0;
unsigned short timeAtStage3 = 0;
bool pixelsOnStage3 = false;
//This array is used for the deflation rate
float last10Pressure[10];
int indexOf10Pressure = 0;
//This float is used later to calculate the rate of change every 100 ms
float oldPressure;

//These are the values to be found
float systolic;
float diastolic;
float maxDifference;
float maxPeak = 0;
float strongestHeartbeat;
float heartRate;
//This bool is gonna be useful to indicate when the first heartbeat has been found
bool systolicFound = false;
float pressureBottomBeat;
float pressureTopPeak;

//This bool is gonna be used to calculate the biggest heartbeat
bool goingUp = false;

//This is gonna help for calculating the heart rate
int heartBeatCounter;
int timeForHeartRate;


float lowPass(){
  float sum = 0;
  for(size_t i = 0; i < len; i++){
    sum += reading[i];
  }
  return ((float)(sum / len));
}

float readPressure(){
  float realPressure = 0;
  uint32_t value = 0;
  Wire.beginTransmission(Pressure);

  Wire.write(Output_Measurement);
  Wire.write(exit);
  Wire.write(exit);

  Wire.endTransmission(false);
  delay(5);

  Wire.requestFrom(Pressure, 4, true);

  if (Wire.available())
  {
    int status = Wire.read();
    
    if(status & (1<<5)){
      Serial.println("device is busy");
      return -1;
    }
  //  else if (status & (1<<6)){
  //    Serial.println("device has power");
  //  }
    else if (status & (1<<2)){
      Serial.println("bad memory");
      return -3;
    }
    else if(status & (1<<0)){
      Serial.println("saturated");
      return -4;
    }
    
    value |= ((uint32_t)Wire.read()<<16);
    value |= ((uint32_t)Wire.read()<<8);
    value |= (Wire.read()<<0);
    reading[indexOfArray] = value;

    
    if(value >= 0){
      //Serial.println(value);
      realPressure = (float)(value - Output_Min) * ((float)Pressure_Max - (float)Pressure_Min)/((float)Output_Max - (float)Output_Min) + Pressure_Min;
      reading[indexOfArray] = realPressure;
      indexOfArray++;
      if(indexOfArray >= len){
        indexOfArray = 0;
      }
      return realPressure;
    }
    
  }
  return (float)value;
}

//This function calculates the deflation rate and light up neo pixels accordingly
void deflationRate(float currPressure){
  //We are gonna calculate the rate of change every second
  //We know that the sample from 10 iterations ago was taken approx 1 second ago
  float previousPressure = last10Pressure[(indexOf10Pressure+1)%10];

    float decreaseRate = abs(currPressure - previousPressure);

    //This means that the decrease rate is too high
    if(decreaseRate > 5){
      CircuitPlayground.clearPixels();
      for(int x = 0; x < 10; x++){
        CircuitPlayground.setPixelColor(x, 0xFF0000);
      }
    }
    //This means that the decrease rate is too low
    else if(decreaseRate < 1){
      CircuitPlayground.clearPixels();
      for(int x = 0; x < 10; x++){
        CircuitPlayground.setPixelColor(x, 0x0000FF);
      }
    }
    //This means the decrease rate is good
    else{
      CircuitPlayground.clearPixels();
      for(int x = 0; x < 10; x++){
        CircuitPlayground.setPixelColor(x, 0x00FF00);
      }
    }

    last10Pressure[indexOf10Pressure] = currPressure;
    indexOf10Pressure = (indexOf10Pressure + 1) % 10;

}

void setup(){
  Serial.begin(9600);
  Wire.begin();
  screen.Begin();
  CircuitPlayground.begin();
  stage = 1;
  CircuitPlayground.setPixelColor(0, 0xFFFF00);
}

void loop(){
  readPressure();
  //Serial.println(readPressure());
  float pressure = lowPass();
  //Serial.println(pressure);
  delay(100);
  //Serial.println(stage);
  //Stage 1 is the beginning of the process when the user hasn't started pumping
  if((stage == 1) ) {
    if(pressure > 30){
      //When user starts pumping, we move on to stage 2 and light up 2 neopixles
      //To indicate pressure is increasing
      CircuitPlayground.setPixelColor(1,0xFFFF00);
      CircuitPlayground.setPixelColor(2,0xFFFF00);
      stage  = 2;
    }
  }

  //Stage 2 is when the user is pumping
  else if((stage == 2)){
    //This stage just lights up neopixels
    //like a progress bar to show the user
    //how much pressure they have pumped

    if((pressure > 70) && (pressure < 100)){
      CircuitPlayground.setPixelColor(3,0xFFFF00);
      CircuitPlayground.setPixelColor(4,0xFFFF00);
    }

    if((pressure > 100) && (pressure < 130)){
      CircuitPlayground.setPixelColor(5,0xFFFF00);
      CircuitPlayground.setPixelColor(6,0xFFFF00);
    }

    if((pressure > 130) && (pressure < 160)){
      CircuitPlayground.setPixelColor(7,0xFFFF00);
      CircuitPlayground.setPixelColor(8,0xFFFF00);
    }

    if(pressure > 160){
      CircuitPlayground.setPixelColor(9,0xFFFF00);
    }

    if(pressure > 170){
      stage = 3;
      CircuitPlayground.clearPixels();
    }
  }
  //Stage 3 is when the user needs to stop pumping 
  //This stage only lasts for a couple seconds
  //The end of this stage means the user needs to start deflating at a decent rate
  else if(stage == 3){
    //Blink the neopixels for a couple seconds to indicate that the user needs to stop pumping
    if(pixelsOnStage3){
      CircuitPlayground.clearPixels();
      pixelsOnStage3 = false;
    }
    else{
      for(int x = 0; x < 10; x++){
        CircuitPlayground.setPixelColor(x, 0xFFFFFF);
      }
      pixelsOnStage3 = true;
    }
    //This is just preparation for the next stage
    //This array is used to calculate deflation rate
    last10Pressure[indexOf10Pressure] = pressure;
    indexOf10Pressure = (indexOf10Pressure + 1) % 10;
    
    if(timeAtStage3 > 20){
      stage = 4;
      CircuitPlayground.clearPixels();
      for(int x = 0; x < 10; x++){
        CircuitPlayground.setPixelColor(x, 0x00FF00);
      }
      //Setting up stuff needed in stage 4
      pixelsOnStage3 = false;
      timeAtStage3 = 0;
      oldPressure = pressure;
      heartBeatCounter = 0;
      timeForHeartRate = 0;
      maxDifference = 0;
    }
    //This is just to keep count that we only spend 
    //a couple seconds on stage 3
    timeAtStage3++;
    
  }
  //Stage 4 is the deflating stage. Here is where the pressure analysis takes place
  else if(stage == 4){
    //This takes care of the deflation rate stuff
    deflationRate(pressure);
    
    float slope = (pressure - oldPressure);
    Serial.print("Slope -> ");
    Serial.println(slope);

    //Before systolic is found, no need to worry about detecting heartbeats
    if(systolicFound){
      //This is the end of a heartbeat
      if(goingUp && (slope < 0)){
        pressureTopPeak = pressure;
        if((pressureTopPeak - pressureBottomBeat) > maxDifference){
          maxDifference = pressureTopPeak - pressureBottomBeat;
          maxPeak = pressure;
        }
        else{
          strongestHeartbeat = maxPeak;
        }
        goingUp = false;
      }
      //This means that a new heartbeat was detected
      else if(!goingUp && (slope > 0)){
        pressureBottomBeat = pressure;
        goingUp = true;
        //Everytime that a new heartbeat starts, you increment the heartbeat counter
        heartBeatCounter++;
      }
      timeForHeartRate++;
    }

    //The program enters this if statement when the first heartbeat/systolic is found
    if((slope > 0) && (!systolicFound)){
      systolic = pressure;
      systolicFound = true;
      goingUp = true;
      pressureBottomBeat = pressure;
      heartBeatCounter++;
    }



    oldPressure = pressure;

    //by the time we're at 50, we already gathered enough data
    //But we can change this later
    if(pressure < 50){
      stage = 5;
      CircuitPlayground.clearPixels();
      for(int x = 0; x < 10; x++){
        CircuitPlayground.setPixelColor(x, 0xFF00FF);
      }
    }
  }
  //Stage 5 is when you tell the user the data of their heart pressure
  else if(stage == 5){
    Serial.print("Your systolic is ");
    Serial.println(systolic);
    diastolic = systolic - strongestHeartbeat;
    Serial.print("Your diastolic is ");
    Serial.println(diastolic);
    float heartRate = heartBeatCounter / (timeForHeartRate*0.001); //Multiply the time by 0.001 because it's in Ms
    Serial.print("Your heart rate is ");
    Serial.println(heartRate);
    delay(10000);

    //After telling user their data, go back to stage 1
    CircuitPlayground.clearPixels();
    CircuitPlayground.setPixelColor(0, 0xFFFF00);
    stage = 1;
  }
    
}
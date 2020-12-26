#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

bool gotReading, newReadingGain;
uint8_t gainSettingTemp, gainSetting; 
float lux, ev;
double hysteresis = 1;

// Different sensitivity settings for the TSL2591 luminosity sensor
void configureSensor(int choice) 
{
Serial.println(F("Config sensor!!!"));
  if (choice == 0) 
  { // Least sensitivity possible
    tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);   // shortest integration time
  } 
  else if (choice == 1) 
  {  
    tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain  
    tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);   // longest integration time
    hysteresis = 0.8;
  } 
  else if (choice == 2) 
  {  
    tsl.setGain(TSL2591_GAIN_MED);    // 25x gain
    tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);   // longest integration time   
    hysteresis = 0.9;
  } 
  else if (choice == 3) 
  { // Highest sensitivity settings
    tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
    tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);   // longest integration time   
    hysteresis = 1.1;
  }

  newReadingGain = true;  // The next measurement will be less accurate, it shouldn't be used.
}


void setup(void) 
{
  Serial.begin(9600);

  if (tsl.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor!"));
  } else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
  
  /* Configure the sensor */
  gainSetting = 2;
  configureSensor(gainSetting);
  newReadingGain = true;
  gotReading = false;
}

// Convert lux to ev, only valid for 100 ASA
float lux2ev(float lux) 
{
	return (log(lux) - log(2.5)) / log(2);
}

/* TSL2591 Autogain based on code by D-Winker :
   https://github.com/D-Winker/TSL2591_AutoAdjust
*/
float sensorReading() 
{
  Serial.println(F("Begin Reading!!!"));
  float newLux;

  while (!gotReading) 
  {
    if (newReadingGain) 
    { // We just changed the gain settings, so we need to take a reading then throw it out.
      tsl.getFullLuminosity();
      newReadingGain = false;
      Serial.println(F("Carbage Reading"));
    } 
    else 
    { // If the gain wasn't just changed.
    Serial.println(F("If the gain wasnt changed"));
      gainSettingTemp = gainSetting;

      // Take a measurement
      uint32_t lum = tsl.getFullLuminosity();  // Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum.    
      newLux = tsl.calculateLux((lum & 0xFFFF), (lum >> 16));
      
      // Adjust the gain if necessary
      if (newLux == 0 || newLux > 60000) 
      {  // The sensor saturated or is close
        if (gainSetting > 0) 
        {
          gainSettingTemp = gainSetting - 1;  // Decrease the gain
        }
      } 
      else if (newLux > 1700 * hysteresis) 
      {
        gainSettingTemp = 1;  // Use low gain setting
      } 
      else if (newLux < 100 * hysteresis) 
      {
        gainSettingTemp = 3;  // Use high gain setting
      } 
      else 
      {
        gainSettingTemp = 2;  // Use normal gain setting
      }

      // Adjust the gain or set the measurements as appropriate
      if (gainSetting == gainSettingTemp) 
      {  // Nothing has changed
        if (newLux < 0) 
        {
          newLux = 0;
        }

        gotReading = true;
        return newLux;
      } 
      else 
      {
        Serial.print("Updating setting "); Serial.print(gainSetting); Serial.print(" to "); Serial.println(gainSettingTemp);
        gainSetting = gainSettingTemp;
        configureSensor(gainSetting);
      }
    }
  }

  gotReading = false;
}

void loop(void) { 
  lux = sensorReading();
  ev = lux2ev(lux);

  Serial.print(F("Lux: ")); Serial.println(lux, 6);
  Serial.print(F("EV: ")); Serial.println(ev, 6);
  
  delay(2000);
}
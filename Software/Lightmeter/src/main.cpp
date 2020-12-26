#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

const double CALIBRATION_INCIDENT = 250;

bool gotReading, newReadingGain;
uint8_t gainSetting; 
double lux, ev;
double hysteresis = 1;

double fStop = 1;
double shutterSpeed = 0.008;
double iso = 2500;


// Different sensitivity settings for the TSL2591 luminosity sensor
void configureSensor(uint16_t gain) 
{

  if (gain == 0) 
  { // Least sensitivity possible
    tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);   // shortest integration time
  } 
  else if (gain == 1) 
  {  
    tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain  
    tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS); 
    hysteresis = 0.8;
  } 
  else if (gain == 2) 
  {  
    tsl.setGain(TSL2591_GAIN_MED);    // 25x gain
    tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);   
    hysteresis = 0.9;
  } 
  else if (gain == 3) 
  {
    tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
    tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS); 
    hysteresis = 1.1;
  }
  else if (gain == 4) 
  {
    tsl.setGain(TSL2591_GAIN_MAX); 
    tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);   
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
double lux2ev(double lux) 
{
	return (log(lux) - log(2.5)) / log(2);
}

/* Calculate the ISO
   https://www.wolframalpha.com/input/?i=solve+s+in+%28n**2%2Ft%29+%3D+%28%28e*s%29%2Fc%29
*/
double calculateIso()
{
  double calculateIsoTemp = (CALIBRATION_INCIDENT * (pow(fStop, 2))) / (lux * shutterSpeed);

  return calculateIsoTemp;
}

/* Calculate the shutter speed
   https://www.wolframalpha.com/input/?i=solve+t+in+%28n**2%2Ft%29+%3D+%28%28e*s%29%2Fc%29
*/
double calculateShutterSpeed()
{
  double shutterSpeedTemp = (CALIBRATION_INCIDENT * (pow(fStop, 2))) / (lux * iso);

  return shutterSpeedTemp;
}

/* Calculate the shutter speed
   https://www.wolframalpha.com/input/?i=solve+n+in+%28n**2%2Ft%29+%3D+%28%28e*s%29%2Fc%29
*/
double calculateFStop()
{
  double fStopTemp = (sqrt(lux) * sqrt(iso) * sqrt(shutterSpeed)) / sqrt(CALIBRATION_INCIDENT);

  return fStopTemp;
}

/* Solve the incident constant
   https://www.wolframalpha.com/input/?i=solve+c+in+%28n**2%2Ft%29+%3D+%28%28e*s%29%2Fc%29
*/
double solveIncidentConstant()
{
  double incidentConstant = (lux * iso * shutterSpeed) / (pow(fStop, 2));

  return incidentConstant;
}

/* TSL2591 Autogain based on code by D-Winker :
   https://github.com/D-Winker/TSL2591_AutoAdjust
*/
double sensorReading() 
{
  uint16_t ir, full;
  uint8_t gainSettingTemp;
  double newLux;

  while(!gotReading) 
  {
    if(newReadingGain) 
    {
      tsl.getFullLuminosity();
      newReadingGain = false;
    }
    else
    {
      uint32_t lum = tsl.getFullLuminosity();
      ir = lum >> 16;
      full = lum & 0xFFFF;
      gainSettingTemp = gainSetting;

      newLux = tsl.calculateLux(full, ir);
      // Interpret the measurement and decide if the gain should be adjusted
      if (newLux <= 0 || newLux > 60000) 
      {  // The sensor saturated or is close
        if(gainSettingTemp != 0)
        {
          gainSettingTemp--;
        }
      } 
      else if (newLux > 1700 * hysteresis) 
      {
        gainSettingTemp = 1;  // Use low gain setting
      }  
      else if (newLux < 100 * hysteresis && newLux > 5 * hysteresis) {
        gainSettingTemp = 3;  // Use high gain setting
      }
      else if (newLux < 5 * hysteresis) 
      {
        gainSettingTemp = 4;  // Use max gain setting
      }
      else 
      {
        gainSettingTemp = 2;  // Use normal gain setting
      }

      // Adjust the gain or set the measurements as appropriate
      if (gainSetting == gainSettingTemp) 
      {  // Nothing has changed
        gotReading = true;
      } 
      else 
      {
        Serial.print("Updating setting "); Serial.print(gainSetting); Serial.print(" to "); Serial.println(gainSettingTemp);
        if(gainSettingTemp <= 4 && gainSettingTemp >= 0) {
          gainSetting = gainSettingTemp;
        }
        configureSensor(gainSetting);
      }
    }
  }
  gotReading = false;
  return newLux;
}

void loop(void) { 
  lux = sensorReading();
  ev = lux2ev(lux);

  Serial.print(F("Lux: ")); Serial.println(lux);
  Serial.print(F("EV: ")); Serial.println(ev);

  Serial.print(F("ISO: ")); Serial.println(calculateIso());

  delay(500);
}
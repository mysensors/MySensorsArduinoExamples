/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 1.1 - AnonymousZebra
 * 
 * DESCRIPTION
 * This is an example of using the Bosch BME280 module, which can measure temperature, humidity and airpressure, 
 * and do so really accurately, while using very little power. A 3.3v and a 5v version is available, make sure to check which one you have.
 * It communicates over the I2C protocol.
 * 
 * This script uses the BME280 library by Embedded Adventures. Download it, and place it in your Arduino library folder. 
 * https://github.com/embeddedadventures/BME280
 * 
 * Connect your sensor's powerlines, and connect your sensor to the SDA and SCL pins of your board. 
 * On Arduino Nano SDA is pin A4, and SCL is pin A5.
 * On the Ardunio Mega and Due the SDA in pin 20, and the SCL is pin 21.
 * On the Arduino Leonardo and Pro Micro 2 the SDA in pin 2, and the SCL is pin 3.
 * 
 * This script has been written in such a way that it can at the same time function as a repeater-node. 
 * It can also easily be used on battery power. Booth features can be turned on in the code below.
 *
 * Finally, you can decide if you want the forecast feature to be turned on. This is a cool feature, 
 * but there is a catch: it also means that you are locked into taking a measurement exactly once a 
 * minute, to build up prediction data for the algorithm.
 * 
 * The reason so many variables have BME280 at the beginning, is so that it is easier to combine multiple sensors on one Arduino.
 *
 *
 * The BME280 datasheet: https://cdn.sparkfun.com/assets/learn_tutorials/4/1/9/BST-BME280_DS001-10.pdf
 *
 */

// if you uncomment this, you can get test and debug updates about everything the sensor is doing by using the serial monitor tool.
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24                            // A 2.4Ghz transmitter and receiver, often used with MySensors.
// #define MY_RF24_PA_LEVEL RF24_PA_MIN              // This sets a low-power mode for the radio. Useful if you use the version with the bigger antenna, but don't want to power that from a separate power source. It can also fix problems with fake Chinese versions of the radio.
// #define MY_RADIO_RFM69                         // 433Mhz transmitter and reveiver.

// Do you want this sensor to also be a repeater?
// #define MY_REPEATER_FEATURE                    // Just remove the two slashes at the beginning of this line to also enable this sensor to act as a repeater for other sensors. If this node is on battery power, you probably shouldn't enable this.

// Are you using this sensor on battery power?
// #define BATTERY_POWERED                        // Just remove the two slashes at the beginning of this line if your node is battery powered. It will then go into deep sleep as much as possible. While it's sleeping it can't work as a repeater!

// Would you like the sensor to generate a weather forecast based on the barometric pressure? 
#define GENERATE_FORECAST			  // Just remove the two slashes at the beginning of this line to enable this feature.


// LIBRARIES
#include <SPI.h>                                  // A communication backbone, the Serial Peripheral Interface.
#include <MySensors.h>                            // The MySensors library. Hurray!
#include <Wire.h>                                 // Enables the Wire communication protocol.
//#include <Adafruit_BME280.h>                    // alternative library you could try (DIY; no code for this is in here yet).
//#include <SparkFunBME280.h>                     // alternative library you could try (DIY; no code for this is in here yet).
#include <BME280_MOD-1022.h>                      // Bosch BME280 Embedded Adventures MOD-1022 weather multi-sensor Arduino code, written originally by Embedded Adventures. https://github.com/embeddedadventures/BME280


// VARIABLES YOU CAN CHANGE
const float ALTITUDE = 14;                        // Change this value to your location's altitude (in m). Use your smartphone GPS to get an accurate value, or use an online map.
unsigned long BME280measurementInterval = 60000;  // Sleep time between reads for the BME sensor (in ms). Keep this value at 60000 if you have enabled the forecast feature, as the forecast algorithm needs a sample every minute.
#define COMPARE_TEMP 1                            // Send temperature only if it changed? 1 = Yes 0 = No. Can save battery.
float tempThreshold = 0.1;                        // How big a temperature difference has to minimally  be before an update is sent. Makes the sensor less precise, but also less jittery, and can save battery.
#define COMPARE_HUM 1                             // Send temperature only if changed? 1 = Yes 0 = No. Can save battery.
float humThreshold = 0.1;                         // How big a humidity difference has to minimally be before an update is sent. Makes the sensor less precise, but also less jittery, and can save battery.
#define COMPARE_BARO 1                            // Send temperature only if changed? 1 = Yes 0 = No. Can save battery.
float presThreshold = 0.1;                        // How big a barometric difference has to minimally be before an update is sent. Makes the sensor less precise, but also less jittery, and can save battery.
            

//VARIABLES YOU PROBABLY SHOULDN'T CHANGE
#define TEMP_CHILD_ID 0				// for MySensors. Within this node each sensortype should have its own ID number.
#define HUM_CHILD_ID 1				// for MySensors. Within this node each sensortype should have its own ID number.
#define BARO_CHILD_ID 2 			// for MySensors. Within this node each sensortype should have its own ID number.
float lastTemperature = -1;			// Stores the previous measurement, so it can be compared with a new measurement.
float lastHumidity = -1;			// Stores the previous measurement, so it can be compared with a new measurement.
float lastPressure = -1;			// Stores the previous measurement, so it can be compared with a new measurement.
unsigned long BME280measurementSleepTime = 0;  	// variable to store the calculated Sleep time if the node is battery powered.
bool metric = true;				// Variable that stores if the sensor will output the temperature in Fahrenheit of Celsius. The gateway sends this preference to the node, so you dont need to change it here.
bool receivedConfig = false;			// The MySensors gateway will tell the node if it should output in metric or not.

#define CONVERSION_FACTOR (1.0/10.0) 		// used by forecast algorithm to convert from Pa to kPa, by dividing hPa by 10.

#ifdef GENERATE_FORECAST 			//  Below you will find a lot of variables used by the forecast algorithm.
const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,     				// "Stable Weather Pattern"
  SUNNY = 1,      				// "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     				// "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   				// "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, 				// "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     				// "Unknown (More Time needed)
};
int lastForecast = -1;				// Stores the previous forecast, so it can be compared with a new forecast.
const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];
int minuteCount = 0;				// Helps the forecst algorithm keep time.
bool firstRound = true;				// Helps the forecast algorithm recognise if the sensor has just been powered up.
float pressureAvg;				// Average value is used in forecast algorithm.
float pressureAvg2;				// Average after 2 hours is used as reference value for the next iteration.
float dP_dt;					// Pressure delta over time
#endif

// MYSENSORS COMMUNICATION VARIABLES
MyMessage temperatureMsg(TEMP_CHILD_ID, V_TEMP);
MyMessage humidityMsg(HUM_CHILD_ID, V_HUM);
MyMessage pressureMsg(BARO_CHILD_ID, V_PRESSURE);
#ifdef GENERATE_FORECAST
MyMessage forecastMsg(BARO_CHILD_ID, V_FORECAST);
#endif


void setup() {
  Wire.begin(); // Wire.begin(sda, scl) // starts the wire communication protocol, used to chat with the BME280 sensor.
  Serial.begin(115200); // for serial debugging over USB.
  Serial.println("Hello world, I am a sensor node.");

#ifdef BATTERY_POWERED // If the node is battery powered, we'll let Sleep take over the scheduling.
   BME280measurementSleepTime = BME280measurementInterval;
   BME280measurementInterval = 0; // When the Arduino is asleep, millis doesn't increment anymore (time stops as it were). To fix this, we'll set the measurement interval time to 1, so that when the arduino wakes up it will immediately try to measure again.
#endif

}


void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BME280 Sensor", "1.1");

  // Tell the MySensors gateway what kind of sensors this node has, and what their ID's on the node are, as defined in the code above.
  present(BARO_CHILD_ID, S_BARO);
  present(TEMP_CHILD_ID, S_TEMP);
  present(HUM_CHILD_ID, S_HUM);
}


void loop() {

  // You should not change these variables:
  static unsigned long previousBME280Millis = 0;  // Used to remember the time that the BME280 sensor was asked for a measurement.
  unsigned long currentMillis = millis();         // The time since the sensor started, counted in milliseconds. This script tries to avoid using the Sleep function, so that it could at the same time be a MySensors repeater.
  static boolean BME280shouldAsk = true;          // This is true when the time is right for a new measurement to be made.
  static boolean BME280justAsked = false;         // This indicates whether we have just asked the sensor module for a measurement, so the receiving part of the code (part 2) should be primed. This two-part construction helps to bridge the time where the BME280 module is busy, without blocking the entire node from doing anything else (like being a repeater, or working with other connected sensor modules).


  // PART 1. If enough time has passed, a new measurement should be taken:
  if (BME280shouldAsk == true && currentMillis - previousBME280Millis >= BME280measurementInterval) {
    previousBME280Millis = currentMillis; // store the current time as the previous measurement start time.
    BME280shouldAsk = false;
    Serial.println("");
    Serial.println("BME280 - Requesting new data from sensor module.");
    BME280.readCompensationParams();    // Need to read the NVM compensation parameters.

#ifdef BATTERY_POWERED
    // After taking the measurement the chip goes back to sleep. This code is only enabled if you enabled BATTERY POWERED at the top of this script.
    // Oversampling settings (os1x, os2x, os4x, os8x or os16x).
    BME280.writeFilterCoefficient(fc_16);       // IIR Filter coefficient, higher numbers avoid sudden changes to be accounted for (such as slamming a door)
    BME280.writeOversamplingPressure(os16x);    // pressure x16
    BME280.writeOversamplingTemperature(os8x);  // temperature x8
    BME280.writeOversamplingHumidity(os8x);     // humidity x8
    BME280.writeMode(smForced);                 // Forced sample.  After taking the measurement the chip goes back to sleep.
#else
    // Normal mode for regular automatic samples
    BME280.writeStandbyTime(tsb_0p5ms);         // tsb = 0.5ms
    BME280.writeFilterCoefficient(fc_16);       // IIR Filter coefficient 16
    BME280.writeOversamplingPressure(os16x);    // pressure x16
    BME280.writeOversamplingTemperature(os8x);  // temperature x8
    BME280.writeOversamplingHumidity(os8x);     // humidity x8
    BME280.writeMode(smNormal);
#endif
  
    // As we exit part 1, in theory BME280.isMeasuring() should now be true.
    BME280justAsked = true;   
  }


  // Part 2. This will trigger if the sensor has just been asked for a measurement, and is also just done figuring out those measurements.
  if(BME280justAsked == true && BME280.isMeasuring() == false) { // 
    BME280justAsked = false; // makes sure we don't do this part again in the next pass through the main loop.
    Serial.println("BME280 - Sensor module has some new values ready:");
      
    // Read out the data - must do this before calling the getxxxxx routines
    BME280.readMeasurements();
  
    float temperature = BME280.getTemperatureMostAccurate();                    // Must get the temperature first.
    float humidity = BME280.getHumidityMostAccurate();				// Get the humidity.
    float pressure_local = BME280.getPressureMostAccurate();                    // Get pressure at current location
    float pressure = pressure_local/pow((1.0 - ( ALTITUDE / 44330.0 )), 5.255); // Adjust to sea level pressure using user altitude
#ifdef GENERATE_FORECAST      
    int forecast = sample(pressure);						// Run the forecast function with a new pressure update.
#endif

    if (!metric) {
      // Convert temperature to fahrenheit
      temperature = temperature * 9.0 / 5.0 + 32.0;
    }

    // Useful for debugging
    Serial.print("BME280 - Temperature = ");
    Serial.print(temperature);
    Serial.println(metric ? " °C" : " °F");
    Serial.print("BME280 - Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("BME280 - Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");
#ifdef GENERATE_FORECAST      
    Serial.print("BME280 - Forecast = ");
    Serial.println(weather[forecast]);
#endif

    // Now, let's send the measurements to the gateway.

    // Send temperature
    if (COMPARE_TEMP == 1 && abs(temperature - lastTemperature) < tempThreshold) { // is the temperature difference bigger than the threshold?
      Serial.print(temperature - lastTemperature);
      Serial.print("- BME280 - Temperature difference too small, so not sending the new measurement to the gateway.\n");
    } else {
      Serial.print("BME280 - Sending the new temperature to the gateway.\n");
      send(temperatureMsg.set(temperature, 1));
      lastTemperature = temperature; // Save new temperatures to be able to compare in the next round.
    } 

    // Send humidity
    if (COMPARE_TEMP == 1 && abs(humidity - lastHumidity) < humThreshold) { // is the humidity difference bigger than the threshold?
      Serial.print(humidity - lastHumidity);
      Serial.println("- BME280 - Humidity difference too small, so not sending the new measurement to the gateway.");
    } else {
      Serial.println("BME280 - Sending the new humidity to the gateway.");
      send(humidityMsg.set(humidity, 1));
      lastHumidity = humidity; // Save new humidity to be able to compare in the next round.
    }

    // Send pressure
    if (COMPARE_TEMP == 1 && abs(pressure - lastPressure) < presThreshold) { // is the pressure difference bigger than the threshold?
      Serial.print(pressure - lastPressure);
      Serial.println("- BME280 - Pressure difference too small, so not sending the new measurement to the gateway.");
    } else {
      Serial.println("BME280 - Sending the new pressure to the gateway.");
      send(pressureMsg.set(pressure, 1));
      lastPressure = pressure; // Save new pressure to be able to compare in the next round.
    }
      
#ifdef GENERATE_FORECAST
    // Send forecast
    if (forecast != lastForecast) {
      Serial.println("BME280 - Sending the latest forecast to the gateway.");      
      send(forecastMsg.set(weather[forecast]));
      lastForecast = forecast;
    }
#endif 

  Serial.println("BME280 - Measurement complete. Going to wait until next measurement.");
  BME280shouldAsk = true; // Ready for the new round.
  }

#ifdef BATTERY_POWERED
  // This code will only be included in the sketch if the BATTERY POWERED feature is enabled.
  if(BME280shouldAsk == true && BME280justAsked == false) { // Both parts are done, so we can let the sensor sleep again.
    unsigned long quicktimecheck = millis(); // To check how much time has passed since the beginning of being awake, and then calculate from that how long to sleep until the next intended measuring time, we need to know how many milliseconds have passed.
    unsigned long sleeptime = BME280measurementSleepTime - (quicktimecheck - previousBME280Millis); // How much time has passed already during the calculating? Subtract that from the intended interval time.
    Serial.println("BME280 - zzzzZZZZzzzzZZZZzzzz");
    sleep (sleeptime);
    Serial.println("BME280 - Waking up.");
  }
#endif

} // end of main loop.



#ifdef GENERATE_FORECAST
// These functions are only included if the forecast function is enables. The are used to generate a weater prediction by checking if the barometric pressure is rising or falling over time.

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++) {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}


// Forecast algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure) {
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185) {
    minuteCount = 6;
  }

  if (minuteCount == 5) {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour 
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { //first time initial 3 hour
      dP_dt = change; //note this is for t = 1 hour
    }
    else {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    } 
    else {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 3; // note this is for t = 3 hour
    } 
    else {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) { //if time is less than 35 min on the first 3 hour interval.
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25)) {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25) {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05))) {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt >(-0.05)) && (dP_dt < 0.05)) {
    forecast = STABLE;
  }
  else {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  //Serial.print(F("BME280 - Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weather[forecast]);

  return forecast;
}
#endif

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
 * DESCRIPTION
 *
 * Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
 * http://www.mysensors.org/build/temp
 *
 * The cool thing about this temperature sensor (pun intended) is thay you can attach multiple Dallas temperature sensors outputs to the same arduino pin. They will all automatically be recognised as separate sensors.
 *
 * At the moment of writing (februari 2017) you need older versions of the Dallas and OneWire libraries. Please check the website or forum to see if this is still the case.
 *
 * Modifications by anonymous user so that it can now simultaneously function as a MySensors repeater.
 */


// if you uncomment this, you can get test and debug updates about everything the sensor is doing by using the serial monitor tool.
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24                            // A 2.4Ghz transmitter and receiver, often used with MySensors.
// #define MY_RF24_PA_LEVEL RF24_PA_MIN           // This sets a low-power mode for the radio. Useful if you use the verison with the bigger antenna, but don't want to power that from a separate power source. It can also fix problems with fake Chinese versions of the radio.
// #define MY_RADIO_RFM69                         // 433Mhz transmitter and reveiver.

// Choose if you want this sensor to also be a repeater.
// #define MY_REPEATER_FEATURE                    // Just remove the two slashes at the beginning of this line to also enable this sensor to act as a repeater for other sensors. If this node is on battery power, you probably shouldn't enable this.

// Are you using this sensor on battery power?
// #define BATTERY_POWERED                        // Just remove the two slashes at the beginning of this line if your node is battery powered. It will then go into deep sleep as much as possible. While it's sleeping it can't work as a repeater!

#include <SPI.h>
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>


// These defines and variables can be changed:
#define COMPARE_TEMP 1                            // Send temperature only if changed? 1 = Yes 0 = No. Can save battery.
#define ONE_WIRE_BUS 3                            // Pin where Dallas sensor(s) is/are connected.
#define maxAttachedDS18B20 16                     // Maximum amount of teperature sensors you can connect to this arduino (16).
unsigned long measurementInterval = 60000;        // Time to wait between reads (in milliseconds).
float tempThreshold = 0.1;                        // The how big a temperature difference has to be before an update is sent. Makes the sensor less precise, but also less jittery, and can save battery.

// You should not change these:
OneWire oneWire(ONE_WIRE_BUS);                    // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);              // Pass the oneWire reference to Dallas Temperature.
float lastTemperature[maxAttachedDS18B20];        // creates an array to hold the previous temperature measurements for each possible sensor.
int numSensors=0;                                 // variable to contain the number of found attached sensors.
unsigned long measurementSleepTime = 0;           // variable to store the calculated Sleep time if the node is battery powered.
bool metric = true;                               // Variable that stores if the sensor will output the temperature in Fahrenheit of Celsius. The gateway sends this preference to the node.
bool receivedConfig = false;                      // This is not used in the code, but perhaps MySensors requires this?


// Mysensors settings
MyMessage msg(0,V_TEMP);                          // Sets up the message format that we'l be sending to the MySensors gateway later.


void before()
{
   sensors.begin();                               // Startup up the OneWire library. It allows multiple sensors to talk over one wire (one pin).
}

void setup()
{
   for(int i=0; i<maxAttachedDS18B20; i++) 
   {
      lastTemperature[i] = 0;  //Pre-filling array with 0's.
   }
   sensors.setWaitForConversion(false); // requestTemperatures() will not block current thread

#ifdef BATTERY_POWERED // If batterypowered, we'll let Sleep take over the scheduling.
   measurementSleepTime = measurementInterval;
   measurementInterval = 1; // When the Arduino is asleep, millis doesn't increment anymore (time stops as it were). To fix this, we'll set the measurement interval time to 1, so that when the arduino wakes up it will immediately try to measure again.
#endif

   Serial.begin(115200); // for serial debugging.
   Serial.print("Hello world, I am a sensor. \n ");
}

void presentation()
{
   sendSketchInfo("Temperature Sensor", "1.2");    // Send the sketch version information to the gateway and Controller
   numSensors = sensors.getDeviceCount();          // Fetch the number of attached temperature sensors
   for (int i=0; i<numSensors && i<maxAttachedDS18B20; i++) {
      present(i, S_TEMP);                          // Present all sensors to controller (16 maximum).
   }
}


void loop()
{

   // You should not change these variables:
   static boolean isMeasuring = true;                        // Used to indicate when the time is right for a new measurement to be made.
   static boolean isCalculating = false;                     // Used to bridge the time that is needed to calculate the temperature values by the Dallas library.
   unsigned long currentMillis = 0;                   // The millisecond clock in the main loop.
   static unsigned long previousMeasurementMillis = 0;       // Used to remember the time of the last temperature measurement.
   static int16_t conversionTime = 0;                        // Used to store the time needed to calculate the temperature from measurements.

   currentMillis = millis(); // The time since the sensor started, counted in milliseconds. This script tries to avoid using the Sleep function, so that it could at the same time be a MySensors repeater.

   // Let's measure the temperature
   if(isMeasuring == true && currentMillis - previousMeasurementMillis >= measurementInterval) { // If we're not calculating, and enough time has passed, we'll start again.
      isMeasuring = false; // We're measuring, so let's take it off our to-do list.
      Serial.print("Starting new measurement(s)\n");
      previousMeasurementMillis = currentMillis; // Mark the time of the initialiation of this measurement.

      // Fetch temperatures from Dallas sensors
      sensors.requestTemperatures();

      // query conversion time. Apparently it takes a while to calculate.
      //ConversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
      conversionTime = millisToWaitForConversion(sensors.getResolution()); // This is a modified version of the line above, to deal with the problem in the current Dallas library.
      isCalculating = true; //Next step is to re-calculate the temperature again.
   }


   // Next, let's calculate and send the temperature
   if(isCalculating == true && currentMillis - conversionTime > previousMeasurementMillis) {
      isCalculating = false; // We're doing this now, so check calculating off the to-do list too.
      for (int i=0; i<numSensors && i<maxAttachedDS18B20; i++){  // Loop through all the attached temperature sensors.   
         float temperature = getControllerConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i); // Fetch the temperature form the current sensor
         Serial.print("Sensor #");
         Serial.print(i);
         Serial.print(" says it is ");
         Serial.print(temperature);
         Serial.print(" degrees\n");
         if(temperature != -127.00 && temperature != 85.00) { // Avoids working with measurement errors.
            if (COMPARE_TEMP == 1 && abs(temperature - lastTemperature[i]) < tempThreshold) { // is the temperature difference bigger than the threshold?
               Serial.print(temperature - lastTemperature[i]);
               Serial.print("- difference too small, so not sending the new measurement to the gateway.\n");
            } else {
               Serial.print(temperature - lastTemperature[i]);
               Serial.print("Sending the new temperature to the gateway.\n");
               send(msg.setSensor(i).set(temperature,1));
               lastTemperature[i] = temperature; // Save new temperatures to be able to compare in the next round.
            }
         }
      }

      // Both tasks are done. Time to wait until we should measure again.
      Serial.print("zzzzZZZZzzzzZZZZzzzz\n");

#ifdef BATTERY_POWERED
      unsigned long quicktimecheck = millis(); // check how much time has passed during the measurement (can be up to 750 milliseconds), and then calculate from that how long to sleep until the next intended measuring time.
      unsigned long sleeptime = measurementSleepTime - (quicktimecheck - previousMeasurementMillis); //How much time has passed already during the calculating? Subtract that from the intended interval time.
      sleep (sleeptime);
#endif

      isMeasuring = true;
   }
}


// This function helps to avoid a problem with the latest Dallas temperature library.
int16_t millisToWaitForConversion(uint8_t bitResolution)
{
   switch (bitResolution) 
   {
     case 9:
        return 94;
     case 10:
        return 188;
     case 11:
        return 375;
     default:
        return 750;
   }
}

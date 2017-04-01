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


// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24                            // A 2.4Ghz transmitter and receiver, often used with MySensors.
#define MY_RF24_PA_LEVEL RF24_PA_MIN              // This sets a low-power mode for the radio. Useful if you use the verison with the bigger antenna, but don' want to power that from a separate source. It can also fix problems with fake Chinese versions of the radio.
//#define MY_RADIO_RFM69                          // 433Mhz transmitter and reveiver.

// Choose if you want this sensor to also be a repeater.
// #define MY_REPEATER_FEATURE                    // Just remove the two slashes at the beginning of this line to also enable this sensor to act as a repeater for other sensors. If this node is on battery power, you probably shouldn't enable this.
// Are you using this sensor on battery power? 
const int BATTERY_POWERED = false;                // Set this to 'true' if your node is battery powered. It will then go into deep sleep as much as possible. But when it' sleeping it can' work as a repeater.

#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>                  
#include <OneWire.h>                              


// These variables can be changed:
#define COMPARE_TEMP 1                            // Send temperature only if changed? 1 = Yes 0 = No. Can save battery.
#define ONE_WIRE_BUS 3                            // Pin where Dallas sensor(s) is/are connected.
#define MAX_ATTACHED_DS18B20 16                   // Maximum amount of teperature sensors you can connect to this arduino (16).
unsigned long MEASUREMENT_INTERVAL = 30000;       // Time to wait between reads (in milliseconds).

// You should not change these:
OneWire oneWire(ONE_WIRE_BUS);                    // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);              // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];      // creates an array to hold the previous temperature measurements for each possible sensor.
int numSensors=0;                                 // variable to contain the number of found attached sensors.
boolean receivedConfig = false;                   // I have no idea what this does.
boolean metric = true;                            // Probably used by the Dallas library as an indication if the temperature should be calculated in Celsius (metric) or Fahrenheit.
boolean CURRENTLY_MEASURING = true;               // Used to indicate when the time is right for a new measurement to be made.
boolean CURRENTLY_CALCULATING = false;            // Used to bridge the time that is needed to calculate the temperature values by the Dallas library.
unsigned long CURRENT_MILLIS = 0;                 // The millisecond clock in the main loop.
unsigned long PREVIOUS_MEASUREMENT_MILLIS = 0;    // Used to remember the time of the last temperature measurement.
unsigned long MEASUREMENT_SLEEP_TIME = 0;         // variable to store the Sleep time if the node is battery powered.
int16_t CONVERSION_TIME = 0;                      // Used to store the time needed to calculate the temperature from measurements.

// Mysensors settings
MyMessage msg(0,V_TEMP);                          // Sets up the message format that we'l be sending to the MySensors gateway later.


void before()
{
  sensors.begin();                                // Startup up the OneWire library. It allows multiple sensors to talk over one wire (one pin).
}

void setup()  
{ 
  for(int i=0;i<MAX_ATTACHED_DS18B20;i++){lastTemperature[i] = 0;} //Pre-filling array with 0's.
  sensors.setWaitForConversion(false);            // requestTemperatures() will not block current thread
  if (BATTERY_POWERED == true){                    
    MEASUREMENT_SLEEP_TIME = MEASUREMENT_INTERVAL; 
    MEASUREMENT_INTERVAL = 1;                    // We'll let Sleep take over the scheduling. When the arduino is asleep, millis doesn't increment anymore (time stops as it were). To fix this, we'l set the measurement interval time to 1, so that when the arduino wakes up it will immediately try to measure again. 
  }
  Serial.begin(115200);                           // for serial debugging.
  Serial.print("Hello world, I am a sensor. \n ");

}

void presentation() {
  sendSketchInfo("Temperature Sensor", "1.2");    // Send the sketch version information to the gateway and Controller
  numSensors = sensors.getDeviceCount();          // Fetch the number of attached temperature sensors  
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
    present(i, S_TEMP);                          // Present all sensors to controller (16 maximum).
  }
}



void loop(){     

  CURRENT_MILLIS = millis(); // The time since the sensor started, counted in milliseconds. This script tries to avoid using the Sleep function, so that it could at the same time be a MySensors repeater.


  // Let's measure the temperature
  if(CURRENTLY_MEASURING == true && CURRENT_MILLIS - PREVIOUS_MEASUREMENT_MILLIS >= MEASUREMENT_INTERVAL){ // If we're not calculating, and enough time has passed, we'll start again.
    CURRENTLY_MEASURING = false; // We're measuring, so let's take it off our to-do list.
    Serial.print("Starting new measurement(s)\n");
    PREVIOUS_MEASUREMENT_MILLIS = CURRENT_MILLIS; // Mark the time of the initialiation of this measurement.
   
    // Fetch temperatures from Dallas sensors
    sensors.requestTemperatures();

    // query conversion time. Apparently it takes a while to calculate.
    //CONVERSION_TIME = sensors.millisToWaitForConversion(sensors.getResolution());
    CONVERSION_TIME = millisToWaitForConversion(sensors.getResolution());
    CURRENTLY_CALCULATING = true; //Next step is to re-calculate the temperature again.
  }


  // Next, let's calculate and send the temperature
  if(CURRENTLY_CALCULATING == true && CURRENT_MILLIS > PREVIOUS_MEASUREMENT_MILLIS + CONVERSION_TIME ){
    CURRENTLY_CALCULATING = false; // check calculating off the to-do list too.
    
    for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {    // Loop through all the attached temperatur sensors.
      float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;   // Fetch and round temperature to one decimal
      Serial.print("Sensor #");
      Serial.print(i);
      Serial.print(" says it is ");
      Serial.print(temperature);
      Serial.print(" degrees\n");      
      if(temperature != -127.00 && temperature != 85.00){ // avoid working with measurement errors.
        if (COMPARE_TEMP == 1 && lastTemperature[i] == temperature){
          Serial.print("Not sending it though, because it's the same temperature as before.\n");
        }
        else
        {
          Serial.print("Sending the temperature to the gateway.\n");
          send(msg.setSensor(i).set(temperature,1));
          lastTemperature[i] = temperature; // Save new temperatures to be able to compare in the next round.
        }
      }  
    }
    
    // Both tasks are done. Time to wait until we should measure again.
    Serial.print("zzzzZZZZzzzzZZZZzzzz\n");
    if(BATTERY_POWERED == true)
      {
      unsigned long quicktimecheck = millis(); // check how much time has passed during the measurement (can be up to 750 milliseconds), and then calculate from that how long to sleep until the next intended measuring time.
      unsigned long sleeptime = MEASUREMENT_SLEEP_TIME - (quicktimecheck - PREVIOUS_MEASUREMENT_MILLIS); //How much time has passed already during the calculating? Subtract that from the intended interval time.
      sleep (sleeptime);
      }
    CURRENTLY_MEASURING = true;   
  }
}


// This function helps to avoid a problem with the latest Dallas temperature library. 
int16_t millisToWaitForConversion(uint8_t bitResolution){ 
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

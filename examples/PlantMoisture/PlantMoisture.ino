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
 * This node can measure the moisture of 6 different plants. It uses the cheap 'capacitive analog 
 * moisture sensor' that you can get for about 3 dollars an Aliexpress or eBay. For example:
 * https://www.aliexpress.com/item/Analog-Capacitive-Soil-Moisture-Sensor-V1-2-Corrosion-Resistant-Z09-Drop-ship/32858273308.html
 * 
 * Each plant' moisture value can also be responded to individually, either by turning on an LED (wire that to the plan, and you can see which one is thirsty) or, if you want, per-plant automated irrigation by connecting a little solenoid..
 * 
 * Todo: Allow the controller to set the threshold values for each plant individually. Unfortunately, Domoticz doesn't support this yet :-(
 * 
 */

//#define MY_SIGNING_SIMPLE_PASSWD "changeme"           // Useful if your MySensors network is encrypted.
#define MY_SPLASH_SCREEN_DISABLED                       // saves a little memory.
//#define MY_DISABLE_RAM_ROUTING_TABLE_FEATURE          // saves a little memory.

//#define MY_NODE_ID 60                                 // Optional. Sets fixed id with controller.
//#define MY_PARENT_NODE_ID 0                           // Optional. Sets fixed id for controller.
//#define MY_PARENT_NODE_IS_STATIC                      // Optional. Sets fixed id for controller.

#define MY_TRANSPORT_WAIT_READY_MS 10000                // try connecting to the gateway for 10 seconds. Otherwise just continue.

// Enable debug prints to serial monitor
// #define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#define MY_RF24_PA_LEVEL RF24_PA_LOW                    // Low power radio setting works better with cheap Chinese radios.

#include <MySensors.h>

#define NUMBEROFSENSORS 6                               // How many sensors are connected?
#define DRYNESSTHRESHOLD 45                             // minimum moisture level that is still ok. A lower value will trigger LED/irrigation.

#define SLEEPTIME 60                                    // sleep time between the sending of data (in SECONDS). Maximum is 254 seconds. Change "byte" to "int" further down in the code if you want more time between sending updates.
unsigned long lastTimeChecked = 0;                      // used by the measurement timer.

MyMessage msg(0, V_LEVEL);                              // used to send data to the gateway.

void before()
{
  for (byte i = 3; i < NUMBEROFSENSORS + 3; i++){       // Set the LED (or irrigation valves) to their initial position. Because Mysensors uses pin 2, we use pin 3 till 8 as output.
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  } 
}

void presentation()
{
	// send the sketch version information to the gateway and Controller
	sendSketchInfo(F("Plant Sensorium"), F("1.0"));
  // present the sensors
  for (byte i=0; i<NUMBEROFSENSORS ; i++) {
    present(i, S_MOISTURE);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Hello world. Warming up the sensors (15 seconds).")); // to avoid weird measurements
  delay(15000);
}

void loop()
{

  static byte measurementCounter = 0;                     // Counts the measurements. Internally the node measures once per second.

  if (millis() - lastTimeChecked > 1000) {
    lastTimeChecked = millis();
    
    Serial.println(F("__________"));
    
    for (int i=0; i<NUMBEROFSENSORS; i++) {               // loop over all the sensors.
      byte shiftedDigitalPin = i + 3;                     // uses pin 3 till 8 as output. Connect some LED's.
    	int16_t moistureLevel = (1023-analogRead(i))/10.23;
      Serial.print(i);
      Serial.print(F(" mosture level: "));
    	Serial.println(moistureLevel);

      if (moistureLevel < DRYNESSTHRESHOLD){              // if the plant doesn' have enough water, turn on the LED/water.
        Serial.print(F("- moisture level is below "));
        Serial.println(DRYNESSTHRESHOLD);
        digitalWrite(shiftedDigitalPin, HIGH);
      }else if (moistureLevel >= DRYNESSTHRESHOLD + 10){   // turn of the water/led if the plant is wet enough.
        digitalWrite(shiftedDigitalPin, LOW);
      }

      if(measurementCounter < NUMBEROFSENSORS){           // During the first 6 seconds the script will send updated data.
        if(measurementCounter == i){                      // it sends sensor 0 at second 0. Sensor 1 at second 1, etc. This keeps the radio happy.
          send(msg.setSensor(i).set(moistureLevel));
        }
      }
      if(measurementCounter > SLEEPTIME){                 // If enough time has passed, the counter is reset, and new data is sent.
        measurementCounter = 0;
      }else{
        measurementCounter++;
      }
    }
    
  }
}

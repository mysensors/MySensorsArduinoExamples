/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
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
 * Version 1.1 - Ivo Pullens, added battery voltage reporting roughly every 24h
 *
 * DESCRIPTION
 * Motion Sensor example using HC-SR501. Requires at least MySensors 2.3.2 !
 * http://www.mysensors.org/build/motion
 * Every 24 hours the battery level will be reported.
 * Battery power will be measured as described here.
 * https://www.mysensors.org/build/humidity_si7021
 */

// Enable debug prints
// #define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#include <MySensors.h>
#include <Vcc.h>			// Install from https://github.com/Yveaux/Arduino_Vcc

const float VccMin        = 1.8;      // Minimum expected Vcc level, in Volts: Brownout at 1.8V    -> 0%
const float VccMax        = 2.0*1.6;  // Maximum expected Vcc level, in Volts: 2xAA fresh Alkaline -> 100%
const float VccCorrection = 1.0;      // Measured Vcc by multimeter divided by reported Vcc
static Vcc vcc(VccCorrection); 

#define SLEEP_TIME_MS (24ul*60ul*60ul*1000ul) // Sleep time between battery reports (24h, in milliseconds)
#define DIGITAL_INPUT_SENSOR 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define CHILD_ID 1   // Id of the sensor child

// Initialize motion message
MyMessage msg(CHILD_ID, V_TRIPPED);

void setup()
{
	pinMode(DIGITAL_INPUT_SENSOR, INPUT);      // sets the motion sensor digital pin as input
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(F("Motion Sensor"), F("1.1"));

	// Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID, S_MOTION, F("Motion detected"));
}

void loop()
{
	static uint32_t sleepTimeMs = SLEEP_TIME_MS;

	// Sleep until interrupt comes in on motion sensor, or if sleepTimeMs elapsed.
	sleep(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), RISING, sleepTimeMs);

	// Read digital motion value
	const bool tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH;

	Serial.print(F("Motion tripped "));
	Serial.println(tripped);
	send(msg.set(tripped?"1":"0"));  // Send tripped value to gw

    // Request how much of the sleep time is remaining
    sleepTimeMs = getSleepRemaining();

	Serial.print(F("Remaining sleep time [ms] "));
	Serial.println(sleepTimeMs);

    if (0ul == sleepTimeMs)
    {
        // Report battery level and restart the cycle
		const uint8_t batteryPcnt = static_cast<uint8_t>(0.5 + vcc.Read_Perc(VccMin, VccMax));
		sendBatteryLevel(batteryPcnt);

		Serial.print(F("Vbat "));
		Serial.print(vcc.Read_Volts());
		Serial.print(F("\tPerc "));
		Serial.println(batteryPcnt);

        sleepTimeMs = SLEEP_TIME_MS;
    }
}



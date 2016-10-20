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
 * Version 1.0 - Changed for MySensors usage by Bart Eversdijk
 * Version 1.1 - Added option to record manual presets up to 240
 * Version 2.0 - Migrated to MySensrors version 2.0
 * 
 * DESCRIPTION
 *
 * IRrecord: record and play back IR signals as a minimal 
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * An IR LED must be connected to the output PWM pin 3.
 *
 *
 * The logic is:
 * If a V_IR_RECORD is received the node enters in record mode and once a valid IR message has been received 
 * it is stored in EEPROM. The first byte of the V_IR_RECORD message will be used as preset ID 
 * 
 * If a V_IR_SEND the IR message beloning to the preset number of the first message byte is broadcasted
 *
 *
 * Version 0.11 September, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_NODE_ID      5

#include <SPI.h>
#include <MySensors.h>

#include <IRremote.h>  // https://github.com/z3t0/Arduino-IRremote/releases   
// OR install IRRemote via "Sketch" -> "Include Library" -> "Manage Labraries..."
// Search for IRRemote b shirif and press the install button

// Arduino pin to connect the IR receiver to
int RECV_PIN     = 8;

#define CHILD_ID  2  

#define MY_RAWBUF  50
const char * TYPE2STRING[] = {
		"UNKONWN",
		"RC5",
		"RC6",
		"NEC",
		"Sony",
		"Panasonic",
		"JVC",
		"SAMSUNG",
		"Whynter",
		"AIWA RC T501",
		"LG",
		"Sanyo",
		"Mitsubishi",
		"Dish",
		"Sharp",
		"Denon"
};
#define Type2String(x)   TYPE2STRING[x < 0 ? 0 : x]
#define AddrTxt          F(" addres: 0x")
#define ValueTxt         F(" value: 0x")
#define NATxt            F(" - not implemented/found")

// Raw or unknown codes requires an Arduino with a larger memory like a MEGA and some changes to store in EEPROM (now max 255 bytes)
// #define IR_SUPPORT_UNKNOWN_CODES
typedef union
{
  struct
  {
    decode_type_t type;            // The type of code
    unsigned long value;           // The data bits if type is not raw
    int           len;             // The length of the code in bits
    unsigned int  address;         // Used by Panasonic & Sharp [16-bits]
  } code;
#ifdef IR_SUPPORT_UNKNOWN_CODES      
  struct
  {
    decode_type_t type;             // The type of code
    unsigned int  codes[MY_RAWBUF];
    byte          count;           // The number of interval samples
  } raw;
#endif
} IRCode;

#define           MAX_STORED_IR_CODES     10
IRCode            StoredIRCodes[MAX_STORED_IR_CODES];

IRrecv            irrecv(RECV_PIN);
IRsend            irsend;
decode_results    ircode;

#define           NO_PROG_MODE 0xFF
byte              progModeId       = NO_PROG_MODE;

// Manual Preset IR values -- these are working demo values
// VERA call: luup.call_action("urn:schemas-arduino-cc:serviceId:ArduinoIr1", "SendIrCode", {Index=15}, <device number>)
// One can add up to 240 preset codes (if your memory lasts) to see to correct data connect the Arduino with this plug in and
// look at the serial monitor while pressing the desired RC button
IRCode PresetIRCodes[] = {
    { { RC5, 0x01,       12, 0 }},  // 11 - RC5 key "1" 
    { { RC5, 0x02,       12, 0 }},  // 12 - RC5 key "2"
    { { RC5, 0x03,       12, 0 }},  // 13 - RC5 key "3"
    { { NEC, 0xFF30CF,   32, 0 }},  // 14 - NEC key "1"
    { { NEC, 0xFF18E7,   32, 0 }},  // 15 - NEC key "2"
    { { NEC, 0xFF7A85,   32, 0 }},  // 16 - NEC key "3"
    { { NEC, 0xFF10EF,   32, 0 }},  // 17 - NEC key "4"
    { { NEC, 0xFF38C7,   32, 0 }},  // 18 - NEC key "5"
    { { RC6, 0x800F2401, 36, 0 }},  // 19 - RC6 key "1" MicroSoft Mulitmedia RC
    { { RC6, 0x800F2402, 36, 0 }}   // 20 - RC6 key "2" MicroSoft Mulitmedia RC
};
#define MAX_PRESET_IR_CODES  (sizeof(PresetIRCodes)/sizeof(IRCode))
#define MAX_IR_CODES (MAX_STORED_IR_CODES + MAX_PRESET_IR_CODES)

MyMessage msgIrReceive(CHILD_ID, V_IR_RECEIVE);
MyMessage msgIrRecord(CHILD_ID, V_IR_RECORD); 

void setup()  
{  
  // Tell MYS Controller that we're NOT recording
  send(msgIrRecord.set(0));
  
  Serial.println(F("Recall EEPROM settings"));
  recallEeprom(sizeof(StoredIRCodes), (byte *)&StoredIRCodes);

  // Start the ir receiver
  irrecv.enableIRIn(); 
  
  Serial.println(F("Init done..."));
}

void presentation () 
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("IR Rec/Playback", "2.0");

  // Register a sensors to gw. Use binary light for test purposes.
  present(CHILD_ID, S_IR);
}

void loop() 
{
  if (irrecv.decode(&ircode)) {
      dump(&ircode);
      if (progModeId != NO_PROG_MODE) {
         // If we are in PROG mode (Recording) store the new IR code and end PROG mode
         if (storeRCCode(progModeId)) {
            Serial.println(F("Stored "));
          
            // If sucessfull RC decode and storage --> also update the EEPROM
            storeEeprom(sizeof(StoredIRCodes), (byte *)&StoredIRCodes);
            progModeId = NO_PROG_MODE;
           
            // Tell MYS Controller that we're done recording
            send(msgIrRecord.set(0));
         }
      } else {
         // If we are in Playback mode just tell the MYS Controller we did receive an IR code
         if (ircode.decode_type != UNKNOWN) {
             if (ircode.value != REPEAT) {
               // Look if we found a stored preset 0 => not found
               byte num = lookUpPresetCode(&ircode);
               if (num) {
                   // Send IR decode result to the MYS Controller
                   Serial.print(F("Found code for preset #"));
                   Serial.println(num);
                   send(msgIrReceive.set(num));
               }
             }
         }
    }
    // Wait a while before receive next IR-code (also block MySensors receiver so it will not interfere with a new message)
    delay(500);
    
    // Start receiving again
    irrecv.resume();
  }
}

void receive(const MyMessage &message) {
    //Serial.print(F("New message: "));
    //Serial.println(message.type);
   
   if (message.type == V_IR_RECORD) { // IR_RECORD V_VAR1
      // Get IR record requets for index : paramvalue
      progModeId = message.getByte() % MAX_STORED_IR_CODES;
      
      // Tell MYS Controller that we're now in recording mode
      send(msgIrRecord.set(1));
      
      Serial.print(F("Record new IR for: "));
      Serial.println(progModeId);
   }
  
   if (message.type == V_IR_SEND) {
      // Send an IR code from offset: paramvalue - no check for legal value
      Serial.print(F("Send IR preset: "));
      byte code = message.getByte() % MAX_IR_CODES;
      if (code == 0) {
        code = MAX_IR_CODES;
      }
      Serial.print(code);
      sendRCCode(code);
   }

   // Start receiving ir again...
   irrecv.enableIRIn(); 
}


byte lookUpPresetCode (decode_results *ircode)
{
    // Get rit of the RC5/6 toggle bit when looking up
    if (ircode->decode_type == RC5)  {
        ircode->value = ircode->value & 0x7FF;
    }
    if (ircode->decode_type == RC6)  {
        ircode->value = ircode->value & 0xFFFF7FFF;
    }
    for (byte index = 0; index < MAX_STORED_IR_CODES; index++)
    {
      if ( StoredIRCodes[index].code.type  == ircode->decode_type &&
           StoredIRCodes[index].code.value == ircode->value       &&
           StoredIRCodes[index].code.len   == ircode->bits)      {
          // The preset number starts with 1 so the last is stored as 0 -> fix this when looking up the correct index
          return (index == 0) ? MAX_STORED_IR_CODES : index;
      }  
    }
    
    for (byte index = 0; index < MAX_PRESET_IR_CODES; index++)
    {
      if ( PresetIRCodes[index].code.type  == ircode->decode_type &&
           PresetIRCodes[index].code.value == ircode->value       &&
           PresetIRCodes[index].code.len   == ircode->bits)      {
          // The preset number starts with 1 so the last is stored as 0 -> fix this when looking up the correct index
          return ((index == 0) ? MAX_PRESET_IR_CODES : index) + MAX_STORED_IR_CODES;
      }  
    }
    // not found so return 0
    return 0;
}
    
// Stores the code for later playback
bool storeRCCode(byte index) {

  if (ircode.decode_type == UNKNOWN) {
#ifdef IR_SUPPORT_UNKNOWN_CODES  
      Serial.println(F("Received unknown code, saving as raw"));
      // To store raw codes:
      // Drop first value (gap)
      // As of v1.3 of IRLib global values are already in microseconds rather than ticks
      // They have also been adjusted for overreporting/underreporting of marks and spaces
      byte rawCount = min(ircode.rawlen - 1, MY_RAWBUF);
      for (int i = 1; i <= rawCount; i++) {
        StoredIRCodes[index].raw.codes[i - 1] = ircode.rawbuf[i]; // Drop the first value
      };
      return true;
#else 
      return false;
    }
#endif

   if (ircode.value == REPEAT) {
       // Don't record a NEC repeat value as that's useless.
       Serial.println(F("repeat; ignoring."));
       return false;
   }
   // Get rit of the toggle bit when storing RC5/6 
   if (ircode.decode_type == RC5)  {
        ircode.value = ircode.value & 0x07FF;
   }
   if (ircode.decode_type == RC6)  {
        ircode.value = ircode.value & 0xFFFF7FFF;
   }

   StoredIRCodes[index].code.type      = ircode.decode_type;
   StoredIRCodes[index].code.value     = ircode.value;
   StoredIRCodes[index].code.address   = ircode.address;      // Used by Panasonic & Sharp [16-bits]
   StoredIRCodes[index].code.len       = ircode.bits;
   Serial.print(F(" value: 0x"));
   Serial.println(ircode.value, HEX);
   return true;
}

void sendRCCode(byte index) {
   IRCode *pIr = ((index <= MAX_STORED_IR_CODES) ? &StoredIRCodes[index % MAX_STORED_IR_CODES] : &PresetIRCodes[index - MAX_STORED_IR_CODES - 1]);
   
#ifdef IR_SUPPORT_UNKNOWN_CODES  
   if(pIr->code.type == UNKNOWN) {
      // Assume 38 KHz
      irsend.sendRaw(pIr->raw.codes, pIr->raw.count, 38);
      Serial.println(F("Sent raw"));
      return;
   }
#endif

   Serial.print(F(" - sent "));
   Serial.print(Type2String(pIr->code.type));
   if (pIr->code.type == RC5) {
       // For RC5 and RC6 there is a toggle bit for each succesor IR code sent alway toggle this bit, needs to repeat the command 3 times with 100 mS pause
       pIr->code.value ^= 0x0800;
       for (byte i=0; i < 3; i++) {
         if (i > 0) { delay(100); } 
         irsend.sendRC5(pIr->code.value, pIr->code.len);
       }
    } 
    else if (pIr->code.type == RC6) {
       // For RC5 and RC6 there is a toggle bit for each succesor IR code sent alway toggle this bit, needs to repeat the command 3 times with 100 mS pause
       if (pIr->code.len == 20) {
              pIr->code.value ^= 0x10000;
       }
       for (byte i=0; i < 3; i++) {
         if (i > 0) { delay(100); } 
         irsend.sendRC6(pIr->code.value, pIr->code.len);
       }
   }
   else if (pIr->code.type == NEC) {
       irsend.sendNEC(pIr->code.value, pIr->code.len);
    } 
    else if (pIr->code.type == SONY) {
       irsend.sendSony(pIr->code.value, pIr->code.len);
    } 
    else if (pIr->code.type == PANASONIC) {
       irsend.sendPanasonic(pIr->code.address, pIr->code.value);
       Serial.print(AddrTxt);
       Serial.println(pIr->code.address, HEX);
    }
    else if (pIr->code.type == JVC) {
       irsend.sendJVC(pIr->code.value, pIr->code.len, false);
    }
    else if (pIr->code.type == SAMSUNG) {
       irsend.sendSAMSUNG(pIr->code.value, pIr->code.len);
    }
    else if (pIr->code.type == WHYNTER) {
       irsend.sendWhynter(pIr->code.value, pIr->code.len);
    }
    else if (pIr->code.type == AIWA_RC_T501) {
       irsend.sendAiwaRCT501(pIr->code.value);
    }
    else if (pIr->code.type == LG || pIr->code.type == SANYO || pIr->code.type == MITSUBISHI) {
       Serial.println(NATxt);
       return;
    }
    else if (pIr->code.type == DISH) {
      // need to repeat the command 4 times with 100 mS pause
      for (byte i=0; i < 4; i++) {
         if (i > 0) { delay(100); } 
           irsend.sendDISH(pIr->code.value, pIr->code.len);
      }
    }
    else if (pIr->code.type == SHARP) {
       irsend.sendSharp(pIr->code.address, pIr->code.value);
       Serial.print(AddrTxt);
       Serial.println(pIr->code.address, HEX);
    }
    else if (pIr->code.type == DENON) {
       irsend.sendDenon(pIr->code.value, pIr->code.len);
    }
    else {
      // No valid IR type, found it does not make sense to broadcast
      Serial.println(NATxt);
      return; 
    }
    Serial.print(" ");
    Serial.println(pIr->code.value, HEX);
}    

// Dumps out the decode_results structure.
void dump(decode_results *results) {
    int count = results->rawlen;
    
    Serial.print(F("Received : "));
    Serial.print(results->decode_type, DEC);
    Serial.print(F(" "));
    Serial.print(Type2String(results->decode_type));
  
    if (results->decode_type == PANASONIC) {	
      Serial.print(AddrTxt);
      Serial.print(results->address,HEX);
      Serial.print(ValueTxt);
    }
    Serial.print(F(" "));
    Serial.print(results->value, HEX);
    Serial.print(F(" ("));
    Serial.print(results->bits, DEC);
    Serial.println(F(" bits)"));
  
    if (results->decode_type == UNKNOWN) {
      Serial.print(F("Raw ("));
      Serial.print(count, DEC);
      Serial.print(F("): "));
  
      for (int i = 0; i < count; i++) {
        if ((i % 2) == 1) {
          Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
        } 
        else {
          Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
        }
        Serial.print(" ");
      }
      Serial.println("");
    }
}

// Store IR record struct in EEPROM   
void storeEeprom(byte len, byte *buf)
{
    saveState(0, len);
    for (byte i = 1; i < min(len, 100); i++, buf++)
    {
       saveState(i, *buf);
    }
}

void recallEeprom(byte len, byte *buf)
{
    if (loadState(0) != len)
    {
       Serial.print(F("Corrupt EEPROM preset values and Clear EEPROM"));
       for (byte i = 1; i < min(len, 100); i++, buf++)
       {
           *buf = 0;
           storeEeprom(len, buf);
       }
       return;
    }
    for (byte i = 1; i < min(len, 100); i++, buf++)
    {
       *buf = loadState(i);
    }
}

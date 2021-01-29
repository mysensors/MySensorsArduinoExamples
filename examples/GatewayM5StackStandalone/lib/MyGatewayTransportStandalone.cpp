/* Following must be included in the sketch
#define MY_REPEATER_FEATURE
#define MY_GATEWAY_FEATURE
#define MY_IS_GATEWAY (true)
#define MY_NODE_TYPE "GW"

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE

// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60ul
*/

#include "MyGatewayTransportStandalone.h"

#include <Arduino.h>
#include <stdint.h>

#include <core/MyProtocol.h>
#include <core/MyTransport.h>
#include <core/MyGatewayTransport.h>
#include <core/MyMessage.h>
#include <core/MyInclusionMode.h>
#include <core/MyIndication.h>

#define EEPROM_LAST_NODE_ID (100)

//last known node states in RAM
nodeState_t nodeStates[255];

//inclusion process vars
bool inclusion = false;
uint32_t inclusionStartTime = 0;
uint8_t nextNodeId = 255;

//messaging vars
mysensors_command_t lastMessageCommand;
uint8_t lastMessageSender;
uint8_t lastMessageSensor;
uint8_t lastMessageType;
uint8_t lastMessageByte;
MyMessage _responseMsg;
uint16_t messageCounter = 0;

uint16_t getMessageCounter()
{
	return messageCounter;
}

uint8_t getLastMessageSender()
{
	return lastMessageSender;
}

uint8_t getLastMessageSensor()
{
	return lastMessageSensor;
}

uint8_t getLastMessageType()
{
	return lastMessageType;
}

bool getTransportInclusionTimeout(uint32_t duration)
{
	return (bool) (millis() - inclusionStartTime > duration);
}

uint8_t getLastNodeId()
{
	uint8_t lastNodeId = loadState(EEPROM_LAST_NODE_ID);
	if ( lastNodeId == 0xFF ) {
		lastNodeId = 0;
	}
	return lastNodeId;
}

uint8_t getnextNodeId()
{
	uint8_t lastNodeId = loadState(EEPROM_LAST_NODE_ID);
	if ( lastNodeId == 0xFF ) {
		lastNodeId = 0;
	}
	lastNodeId ++;
	saveState(EEPROM_LAST_NODE_ID, lastNodeId);
	return lastNodeId;
}

void resetTransport()
{
	saveState(EEPROM_LAST_NODE_ID, 0);
}

void setTransportInclusion(bool include)
{
	inclusion = include;
	inclusionModeSet(inclusion);
	inclusionStartTime = millis();
	delay(100);
	if ( !include ) {
		nextNodeId = 255;
	}
}

bool getTransportInclusion()
{
	return inclusion;
}

nodeState_t& getNodeState(uint8_t nodeId)
{
	return nodeStates[nodeId];
}

void gatewayTransportReceived(const MyMessage &message)
{
	messageCounter ++;

	lastMessageCommand = message.getCommand();
	lastMessageSender = message.getSender();
	lastMessageSensor = message.getSensor();
	lastMessageType = message.getType();
	lastMessageByte = message.getByte();

	if ( lastMessageSensor == 1 && lastMessageType == 0 ) { //temp
		nodeStates[lastMessageSender].temp = message.getFloat();
	}

	if ( lastMessageSensor == 201 && lastMessageType == 38 ) { //battery
		nodeStates[lastMessageSender].voltage = message.getFloat();
	}

	nodeStates[lastMessageSender].time = millis();

	//notify that a message has been received
	received(lastMessageSender);
}

bool gatewayTransportSend(MyMessage &message)
{
	gatewayTransportReceived(message);
	setIndication(INDICATION_GW_TX);
	return true;
}

bool gatewayTransportInit(void)
{
	inclusion = false;
	inclusionInit();
	inclusionModeSet(false);
	inclusionStartTime = millis();

	for ( uint8_t i = 0; ; i ++ ) {
		nodeStates[i].voltage = INVALID_F;
		nodeStates[i].temp = INVALID_F;
		nodeStates[i].time = 0;
		if ( i == 255 )
			break;
	}
	return true;
}

bool gatewayTransportAvailable(void)
{
	bool result = false;
	if ( ( lastMessageCommand == C_INTERNAL ) && inclusion ) {
		result = true;

		_responseMsg.clear();
		_responseMsg.setCommand(C_INTERNAL);
		_responseMsg.setSender(getNodeId());
		_responseMsg.setDestination(lastMessageSender);

		switch ( lastMessageType ) {
		case I_REGISTRATION_REQUEST: //presumably last in the inclusion sequence
			setIndication(INDICATION_GW_RX);
			_responseMsg.setType(I_REGISTRATION_RESPONSE);
			_responseMsg.set((bool) true);
			break;
		case I_ID_REQUEST:
			_responseMsg.setType(I_ID_RESPONSE);
			_responseMsg.setSensor(AUTO);
			nextNodeId = nextNodeId == 255 ? getnextNodeId() : nextNodeId;
			if ( nextNodeId != 0 ) { //sanity check for free Ids available
				_responseMsg.set((const uint8_t) nextNodeId);
			}
			break;
		case I_FIND_PARENT_REQUEST:
			_responseMsg.setType(I_FIND_PARENT_RESPONSE);
			_responseMsg.set((const uint8_t) 0);
			break;
		case I_DISCOVER_REQUEST:
			_responseMsg.setType(I_DISCOVER_RESPONSE);
			_responseMsg.set((const uint8_t) 0);
			break;
		case I_PING:
			_responseMsg.setType(I_PONG);
			_responseMsg.set((const uint8_t) lastMessageByte++);
			break;
		case I_CONFIG:
			_responseMsg.setType(I_CONFIG);
			_responseMsg.set((const uint8_t) 0);// metric
			break;
		default:
			result = false;
		}
		lastMessageCommand = C_INVALID_7;//mark as handled
	}
	return result;
}

MyMessage& gatewayTransportReceive(void)
{
	return _responseMsg;
}
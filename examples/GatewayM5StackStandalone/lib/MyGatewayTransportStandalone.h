#ifndef MyGatewayTransportStandalone_h
#define MyGatewayTransportStandalone_h

#include <Arduino.h>
#include <stdint.h>

const float INVALID_F = -1000;

typedef struct {
	float voltage;
	float temp;
	uint32_t time;
} nodeState_t;

extern void received(uint8_t nodeId);

bool getTransportInclusionTimeout(uint32_t duration);

uint8_t getLastNodeId();

uint8_t getNextNodeId();

void resetTransport();

void setTransportInclusion(bool include);

bool getTransportInclusion();

nodeState_t& getNodeState(uint8_t nodeId);

uint16_t getMessageCounter();

uint8_t getLastMessageSender();

uint8_t getLastMessageSensor();

uint8_t getLastMessageType();

#endif
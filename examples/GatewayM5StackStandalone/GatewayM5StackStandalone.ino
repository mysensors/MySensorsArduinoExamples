#define MY_DEBUG

#define MY_RADIO_RF24
#define MY_RF24_CE_PIN	(SCL)
#define MY_RF24_CS_PIN	(SDA)
#define MY_RF24_PA_LEVEL RF24_PA_LOW

#define MY_REPEATER_FEATURE
#define MY_GATEWAY_FEATURE
#define MY_IS_GATEWAY (true)
#define MY_NODE_TYPE "GW"

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE

// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60

// Replace default LED handler with TFT display indication
#define MY_INDICATION_HANDLER
#define TFT_IND_W (20)
#define TFT_IND_H (20)
#define TFT_IND_X (320 - 3 * TFT_IND_W)
#define TFT_IND_Y (240 - 1 * TFT_IND_H)

#include <Arduino.h>
#include <stdint.h>

#include <M5Stack.h>
#include <MySensors.h>

// Include from subfolders to avoid funny linking issues from Arduino
#include "lib/MyGatewayTransportStandalone.h"
#include "lib/MyGatewayTransportStandalone.cpp"

uint8_t nodeIndex = 0;

void indication( const indication_t ind )
{
	if ((INDICATION_TX == ind) || (INDICATION_GW_TX == ind)) {
		M5.Lcd.fillRect(TFT_IND_X + 0 * TFT_IND_W, TFT_IND_Y, TFT_IND_W, TFT_IND_H, BLUE);
		delay(25);
		M5.Lcd.fillRect(TFT_IND_X + 0 * TFT_IND_W, TFT_IND_Y, TFT_IND_W, TFT_IND_H, TFT_BLACK);
	} else if ((INDICATION_RX == ind) || (INDICATION_GW_RX == ind)) {
		M5.Lcd.fillRect(TFT_IND_X + 1 * TFT_IND_W, TFT_IND_Y, TFT_IND_W, TFT_IND_H, GREEN);
		delay(25);
		M5.Lcd.fillRect(TFT_IND_X + 1 * TFT_IND_W, TFT_IND_Y, TFT_IND_W, TFT_IND_H, TFT_BLACK);
	} else if (ind > INDICATION_ERR_START) {
		M5.Lcd.fillRect(TFT_IND_X + 2 * TFT_IND_W, TFT_IND_Y, TFT_IND_W, TFT_IND_H, RED);
		delay(25);
		M5.Lcd.fillRect(TFT_IND_X + 2 * TFT_IND_W, TFT_IND_Y, TFT_IND_W, TFT_IND_H, TFT_BLACK);
	}
}

void startScreen()
{
	M5.Lcd.fillScreen(TFT_BLACK);
	M5.Lcd.setCursor(0, 0);
	M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
	M5.Lcd.print (" Status: Ready\n");
	M5.Lcd.printf("Channel: %d\n", MY_RF24_CHANNEL);
	M5.Lcd.printf("  Nodes: %d\n", getLastNodeId());
}

void setup()
{
	//Set up screen and rotate vertically as SPI pins on M5Stack are on the bottom
	//
	// /----- [ SPI ] -----\
	// |-------------------|
	// | [ C ] [ B ] [ A ] |
	// | /---------------\ |
	// | |               | |
	// | |               | |
	// | |               | |
	// | |               | |
	// | \---------------/ |
	// \-------------------/
	M5.begin();
	M5.Lcd.setTextSize(3);
	M5.Lcd.setRotation(3);

	startScreen();
	setTransportInclusion(false);
}

void presentation()
{
	// Present locally attached sensors
}

void setInclusion(bool include)
{
	if ( getTransportInclusion() && getLastNodeId() == 255 ) {
		//No available Ids in the network
		M5.Lcd.setCursor(0, 220);
		M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
		M5.Lcd.print("Nwk full");
	} else {
		setTransportInclusion(include);
	}
}

bool getInclusion()
{
	return getTransportInclusion();
}

void resetGateway()
{
	for (uint16_t i=0; i<EEPROM_LOCAL_CONFIG_ADDRESS; i++) {
		hwWriteConfig(i, 0xFF);
	}
	resetTransport();
}

void loop()
{
	//Call M5 lib to keep track of button states
	M5.update();

	//M5.BtnA/B/C.pressedFor(uint32_t ms);isPressed();wasReleasedFor(uint32_t ms);wasPressed();isPressed();
	if (M5.BtnA.pressedFor(5000)) {//hold button A to reset gateway
		M5.Lcd.fillScreen(TFT_BLACK);
		M5.Lcd.setCursor(0, 0);
		M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
		M5.Lcd.print("Resetting...\n");
		resetGateway();
		M5.Lcd.print("Reset done.\n");
		delay(1000);
		startScreen();
		setTransportInclusion(false);
	} else if (M5.BtnB.wasPressed()) {
		setInclusion(!getInclusion());
	} else if (M5.BtnC.wasPressed()) {//cycle back and print node state
		nodeIndex --;
		nodeIndex = nodeIndex > getLastNodeId() ? getLastNodeId() : nodeIndex;
		printNodeState(nodeIndex, false);
	} else if (M5.BtnA.wasPressed()) {//cycle forward and print node state
		nodeIndex ++;
		nodeIndex = nodeIndex > getLastNodeId() ? 0 : nodeIndex;
		printNodeState(nodeIndex, false);
	}

	if ( getInclusion() && getTransportInclusionTimeout(MY_INCLUSION_MODE_DURATION * 1000) ) {
		setInclusion(false);
	}

	M5.Lcd.setCursor(0, 220);
	if ( getInclusion() ) {
		M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
		M5.Lcd.print("Include ");
	} else {
		M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
		M5.Lcd.print("Normal  ");
	}
}


//Print some common sensor values like voltage and temperature
void printNodeState(uint8_t node, bool last)
{
	M5.Lcd.fillScreen(TFT_BLACK);
	M5.Lcd.setCursor(0, 0);
	M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
	
	nodeState_t nodeState = getNodeState(node);

	if ( last ) {
		M5.Lcd.printf("Sender: %d\n", getLastMessageSender());
		M5.Lcd.printf("Sensor: %d\n", getLastMessageSensor());
		M5.Lcd.printf("  Type: %d\n", getLastMessageType());
	} else {
		M5.Lcd.printf("Sender: %d\n", node);
		if ( nodeState.time > 0 ) {
			uint32_t nowMillis = millis() - nodeState.time;
			uint32_t seconds = nowMillis / 1000;
			int days = seconds / 86400;
			seconds %= 86400;
			byte hours = seconds / 3600;
			seconds %= 3600;
			byte minutes = seconds / 60;
			seconds %= 60;
			M5.Lcd.printf("Ago: %02dh:%02dm:%02ds\n", hours, minutes, seconds);
		} else {
			M5.Lcd.printf("   Ago: N/A\n", node);
		}
		M5.Lcd.printf("\n");
	}
	M5.Lcd.printf("\n");

	M5.Lcd.setTextColor(DARKCYAN, TFT_BLACK);
	M5.Lcd.printf(" Volts: ");
	if ( nodeState.voltage != INVALID_F )
		M5.Lcd.printf("%2.2f", nodeState.voltage);
	M5.Lcd.printf("\n");

	M5.Lcd.setTextColor(DARKGREEN, TFT_BLACK);
	M5.Lcd.printf("  Temp: ");
	if ( nodeState.temp != INVALID_F )
		M5.Lcd.printf("%2.2f", nodeState.temp);
	M5.Lcd.printf("\n");
	
	M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
	M5.Lcd.printf("\n");
	M5.Lcd.printf("\n");
	M5.Lcd.printf("Total Rx: %d\n", getMessageCounter());
}

void received(uint8_t nodeId)
{
	printNodeState(nodeId, true);
}
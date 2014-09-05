//  The MIT License (MIT)
//  
//  Copyright (c) 2014 Henrik Olsson
//  
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//  
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//  
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.

/***************************************************************
 * 
 * Do-It-Yourself TRAFFIC COUNTER
 * 
 ***************************************************************/
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <EEPROM.h>

#include "utility/debug.h"

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS,
ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER); // you can change this clock speed but DI

#define WLAN_SSID       "myNetwork"        // cannot be longer than 32 characters!
#define WLAN_PASS       "myPassword"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define MAC_LENGTH      6
#define MAC_HEX_LENGTH  MAC_LENGTH * 2 + 1

char macHex[MAC_HEX_LENGTH];

#define PLACE_NAME_LENGTH      32
char placeName[PLACE_NAME_LENGTH];

char* topic = (char*) malloc(1);

// byte server[] = { 172, 16, 0, 2 };
char server[] = "aceone.se";

PubSubClient client(server, 1883, callback, cc3000);

#define THRESHOLD       20 //change this amount if necessary. tunes sensitivity.
#define WHEEL_DELAY     50 //number of milliseconds to create accurate readings for cars. prevents bounce.
#define WHEEL_SPACEING  2.7500 //average spacing between wheels of car (METERS)
#define CAR_TIMEOUT     3000

int triggerValue; // pressure reading THRESHOLD for identifying a bike is pressing.
int max = 0;
int isMeasuring = 0;
int countThis = 0;
int strikeNumber = 0;
float firstWheel = 0.0000000;
float secondWheel = 0.0000000;
float wheelTime = 0.0000000;
float speed = 0.0000000;

void setup() {
	pinMode(A0, INPUT);
	pinMode(8, INPUT_PULLUP);
	pinMode(9, OUTPUT);
	Serial.begin(115200);

	Serial.println();
	eeprom_read_string(0, placeName, PLACE_NAME_LENGTH);
      cc3000.begin(false, true);
  if (!cc3000.startSmartConfig())
  {
    Serial.println(F("SmartConfig failed"));
    while(1);
  }

	buildTopic();
	
	Serial.print(F("Conecting to WIFI ."));
	digitalWrite(9, HIGH);

	if (!cc3000.begin()) {
		Serial.println();
		Serial.println(F("Faild to begin"));
		while (1)
			;
	}

	Serial.print(F("."));

	if (!cc3000.deleteProfiles()) {
		while (1)
			;
	}

	Serial.print(F("."));
	char *ssid = WLAN_SSID; /* Max 32 chars */
	if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
		while (1)
			;
	}

	Serial.print(F("."));
	while (!cc3000.checkDHCP()) {
		Serial.print(F("."));
		delay(100); // ToDo: Insert a DHCP timeout!
	}

	Serial.println(F(" OK"));
	digitalWrite(9, LOW);

	uint8_t macAddress[MAC_LENGTH];

	cc3000.getMacAddress(macAddress);
	bytesToHexString(macAddress, macHex, MAC_LENGTH);

	// read local air pressure and create offset.
	triggerValue = analogRead(A0) + THRESHOLD;
	delay(1000);
	Serial.println(F("Traffic Counter"));
	Serial.println(F("___________________________________________________"));
	Serial.println();
	Serial.print(F("Place: "));
	Serial.println(placeName);
	Serial.print(F("Local Air Pressure: "));
	Serial.println(triggerValue - THRESHOLD);
	Serial.println(F("___________________________________________________"));

	String tmpName = "TC-";
	tmpName.concat(macHex);
	char mqttName[tmpName.length()];
	tmpName.toCharArray(mqttName, tmpName.length() + 1);
	Serial.print("'");
	Serial.print(mqttName);
	Serial.println("'");
	if (client.connect(mqttName)) {
		client.publish("traffic/counter/start", mqttName);
		client.publish("traffic/counter/place", placeName);
		client.subscribe("traffic/counter/config");
		Serial.print(F("Conneted to: "));
		Serial.println(server);
	}
	Serial.print("Free RAM: ");
	Serial.println(getFreeRam(), DEC);

}

void loop() {
	//Serial.println(analogRead(A0));

	client.loop();

	//1 - TUBE IS PRESSURIZED INITIALLY
	if (analogRead(A0) > triggerValue) {
		if (strikeNumber == 0 && isMeasuring == 0) { // FIRST HIT
			Serial.println();
			Serial.println(F("Car HERE. "));
			firstWheel = millis();
			isMeasuring = 1;
		}
		if (strikeNumber == 1 && isMeasuring == 1) { // SECOND HIT
			Serial.println(F("Car GONE."));
			secondWheel = millis();
			isMeasuring = 0;
		}
	}

	//2 - TUBE IS STILL PRESSURIZED
	while (analogRead(A0) > max && isMeasuring == 1) { //is being pressed, in all cases. to measure the max pressure.
		max = analogRead(A0);
	}

	//3 - TUBE IS RELEASED
	if (analogRead(A0) < triggerValue - 1 && countThis == 0) { //released by either wheel
		if (strikeNumber == 0 && isMeasuring == 1
				&& (millis() - firstWheel > WHEEL_DELAY)) {
			strikeNumber = 1;
		}
		if (strikeNumber == 1 && isMeasuring == 0
				&& (millis() - secondWheel > WHEEL_DELAY)) {
			countThis = 1;
		}
	}

	//4 - PRESSURE READING IS ACCEPTED AND RECORDED
	if ((analogRead(A0) < triggerValue - 1)
			&& ((countThis == 1 && isMeasuring == 0)
					|| ((millis() - firstWheel) > CAR_TIMEOUT)
							&& isMeasuring == 1)) { //has been released for enough time.
		Serial.print(F("Pressure Reached = "));
		Serial.println(max);
		//Serial.print("time between wheels = ");
		wheelTime = ((secondWheel - firstWheel) / 3600000);
		//Serial.println(wheelTime);
		int time = ((millis() / 1000) / 60) + 1; // the number of seconds since first record.
		speed = (WHEEL_SPACEING / 1000) / wheelTime;
		if (speed > 0) {
			Serial.print(F("Estimated Speed (km/h) = "));
			Serial.println(speed);
		} else {
			speed = 1.0;
		}
		char tmp[6];
		dtostrf(speed, 1, 2, tmp);
		if(client.connected()){
			client.publish(topic, tmp);
		}

		//RESET ALL VALUES
		max = 0;
		strikeNumber = 0;
		countThis = 0;
		isMeasuring = 0;

	}
}

void buildTopic(){
	Serial.print("Free RAM: ");
	Serial.println(getFreeRam(), DEC);
	free(topic);
	Serial.print("Free RAM: ");
	Serial.println(getFreeRam(), DEC);
	String tmpTopic = "traffic/counter/car/";
	tmpTopic.concat(placeName);
	topic = (char*) malloc(tmpTopic.length());
	tmpTopic.toCharArray(topic, tmpTopic.length() + 1);
	Serial.print("Free RAM: ");
	Serial.println(getFreeRam(), DEC);
	Serial.println(topic);
}

void callback(char* topic, byte* payload, unsigned int length) {
	// handle message arrived
	if (length < MAC_HEX_LENGTH) {
		return;
	}

	for (int i = 0; i < MAC_HEX_LENGTH - 1; i++) {
		if (payload[i] != macHex[i]) {
			return;
		}
	}

	int len = length - MAC_HEX_LENGTH + 1;
	// Allocate the correct amount of memory for the payload copy
	char* p = (char*) malloc(len);
	int i;
	// Copy the payload to the new buffer
	for (i = 0; i < len; i++) {
		p[i] = payload[MAC_HEX_LENGTH + i];
	}
	p[i] = '\0';
	eeprom_write_string(0, p);
	memcpy(placeName, p, i + 1);
	// Free the memory
	free(p);
	buildTopic();
}

static char nibbleToChar(uint8_t nibble) {
	if (nibble >= 0 && nibble < 10) {
		return '0' + nibble;
	}
	return 'A' + nibble - 10;
}

/* Convert a buffer of binary values into a hex string representation */
void bytesToHexString(uint8_t *bytes, char *retval, int buflen) {
	int i;
	for (i = 0; i < buflen; i++) {
		retval[i * 2] = nibbleToChar(bytes[i] >> 4);
		retval[i * 2 + 1] = nibbleToChar(bytes[i] & 0x0f);
	}
	retval[i * 2] = '\0';
}

//
// Writes a string starting at the specified address.
// Returns true if the whole string is successfully written.
// Returns false if the address of one or more bytes
// fall outside the allowed range.
// If false is returned, nothing gets written to the eeprom.
//
boolean eeprom_write_string(int addr, char* string) {
	// actual number of bytes to be written
	int numBytes;

	// we'll need to write the string contents
	// plus the string terminator byte (0x00)
	numBytes = strlen(string) + 1;
	for (int i = 0; i < numBytes; i++) {
		EEPROM.write(addr + i, string[i]);
	}
	return true;
}

//
// Reads a string starting from the specified address.
// Returns true if at least one byte (even only the
// string terminator one) is read.
// Returns false if the start address falls outside
// or declare buffer size os zero.
// the allowed range.
// The reading might stop for several reasons:
// - no more space in the provided buffer
// - last eeprom address reached
// - string terminator byte (0x00) encountered.
// The last condition is what should normally occur.
//
boolean eeprom_read_string(int addr, char* buffer, int bufSize) {
	// byte read from eeprom
	byte ch;

	// number of bytes read so far
	int bytesRead;

	// how can we store bytes in an empty buffer ?
	if (bufSize == 0) {
		return false;
	}

	// is there is room for the string terminator only,
	// no reason to go further
	if (bufSize == 1) {
		buffer[0] = 0;
		return true;
	}

	// initialize byte counter
	bytesRead = 0;

	// read next byte from eeprom
	ch = EEPROM.read(addr + bytesRead);

	// store it into the user buffer
	buffer[bytesRead] = ch;

	// increment byte counter
	bytesRead++;

	// stop conditions:
	// - the character just read is the string terminator one (0x00)
	// - we have filled the user buffer
	// - we have reached the last eeprom address
	while ((ch != 0x00) && (bytesRead < bufSize)) {
		// if no stop condition is met, read the next byte from eeprom
		ch = EEPROM.read(addr + bytesRead);

		// store it into the user buffer
		buffer[bytesRead] = ch;

		// increment byte counter
		bytesRead++;
	}

	// make sure the user buffer has a string terminator
	// (0x00) as its last byte
	if ((ch != 0x00) && (bytesRead >= 1)) {
		buffer[bytesRead - 1] = 0;
	}
	return true;
}

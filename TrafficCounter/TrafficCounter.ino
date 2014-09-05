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
 * TRAFFIC COUNTER
 * 
 ***************************************************************/
#include <avr/sleep.h>
#include <avr/power.h>
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include "utility/debug.h"

#define PRESSURE_READING_PIN		A0
#define CONFIGURE_BUTTON_PIN	8
#define LED_INDICATOR_PIN		9
#define PRESSURE_IRQ			0
#define PRESSURE_IRQ_PIN		2

#define MQTT_PORT 				1883
#define PLACE_NAME_LENGTH      	32

#define PUBLISH_DELAY      		3000
#define CONNECTION_ALIVE_TIME	60000

#define THRESHOLD       20 //change this amount if necessary. tunes sensitivity.
#define WHEEL_DELAY     50 //number of milliseconds to create accurate readings for cars. prevents bounce.
#define WHEEL_SPACEING  2.75 //average spacing between wheels of car (METERS)
#define CAR_TIMEOUT     3000
#define SPEED_BUFFER_SIZE     10

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11

#define WLAN_SSID       "C64"        // cannot be longer than 32 characters!
#define WLAN_PASS       "feelgood"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define MAC_LENGTH      6
#define MAC_HEX_LENGTH  MAC_LENGTH * 2 + 1

char macHex[MAC_HEX_LENGTH];

char mqttName[MAC_HEX_LENGTH + 3];
char placeName[PLACE_NAME_LENGTH];

char* topic = (char*) malloc(1);

// byte server[] = { 172, 16, 0, 2 };
char server[] = "aceone.se";

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);
PubSubClient client(server, MQTT_PORT, callback, cc3000);

int triggerValue; // pressure reading THRESHOLD for identifying a bike is pressing.
int max = 0;
int isMeasuring = 0;
int countThis = 0;
int strikeNumber = 0;
float firstWheel = 0.0;
float secondWheel = 0.0;
float wheelTime = 0.0;
float speed = 0.0;
float speed_buffer[SPEED_BUFFER_SIZE];
int speed_buffer_start = 0;
int speed_buffer_count = 0;

unsigned long timeStamp;

void setup() {
	pinMode(PRESSURE_READING_PIN, INPUT);
//	pinMode(CONFIGURE_BUTTON_PIN, INPUT_PULLUP);
	pinMode(LED_INDICATOR_PIN, OUTPUT);
	pinMode(PRESSURE_IRQ_PIN, INPUT);

	Serial.begin(115200);
	 Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
	 
	Serial.println();
	eeprom_read_string(0, placeName, PLACE_NAME_LENGTH);
	buildTopic();

	Serial.print(F("Conecting to WIFI ."));
	digitalWrite(LED_INDICATOR_PIN, HIGH);

	if (!cc3000.begin()) {
		Serial.println();
		Serial.println(F("Faild to begin"));
		while (1)
			;
	}

	Serial.print(F("."));

	Serial.println(F(" OK"));

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
	mqttName[tmpName.length()];
	tmpName.toCharArray(mqttName, tmpName.length() + 1);
	Serial.print(F("Mqtt name: "));
	Serial.println(mqttName);

	while (!enableConnection()) {
		delay(100);
	}
	timeStamp = millis();

	digitalWrite(LED_INDICATOR_PIN, LOW);
//	delay(10000);
//	Serial.println(F("Shut down!"));
//	shutdownConnection();
//	delay(10000);
//	Serial.println(F("Sleep now!"));
//	delay(100);
//	sleepNow();
	 Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
	 
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
		if (strikeNumber == 0 && isMeasuring == 1 && (millis() - firstWheel > WHEEL_DELAY)) {
			strikeNumber = 1;
		}
		if (strikeNumber == 1 && isMeasuring == 0 && (millis() - secondWheel > WHEEL_DELAY)) {
			countThis = 1;
		}
	}

	//4 - PRESSURE READING IS ACCEPTED AND RECORDED
	if ((analogRead(A0) < triggerValue - 1) && ((countThis == 1 && isMeasuring == 0) || ((millis() - firstWheel) > CAR_TIMEOUT) && isMeasuring == 1)) { //has been released for enough time.
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
		publish(speed);
		//RESET ALL VALUES
		max = 0;
		strikeNumber = 0;
		countThis = 0;
		isMeasuring = 0;
	}
	unsigned long ml = millis();
	if (connected() && ml > timeStamp + CONNECTION_ALIVE_TIME) {
		shutdownConnection();
	}
	if (!bufferEmpty() && ml > timeStamp + PUBLISH_DELAY) {
		if (connected()) {
			publishBuffer();
		} else {
			if (!enableConnection()) {
				delay(100);
			}
		}
	}
}

void publishBuffer() {
	while(!bufferEmpty()){
		float speed = bufferRead();
		char tmp[6];
		dtostrf(speed, 1, 2, tmp);
		client.publish(topic, tmp);
	}
	timeStamp = millis();
}

void publish(float speed) {
	timeStamp = millis();
	if (connected()) {
		char tmp[6];
		dtostrf(speed, 1, 2, tmp);
		client.publish(topic, tmp);
	} else {
		bufferWrite(speed);
	}
}

int bufferFull() {
	return speed_buffer_count == SPEED_BUFFER_SIZE;
}

int bufferEmpty() {
	return speed_buffer_count == 0;
}

void bufferWrite(float speed) {
	int end = (speed_buffer_start + speed_buffer_count) % SPEED_BUFFER_SIZE;
	speed_buffer[end] = speed;
	if (speed_buffer_count == SPEED_BUFFER_SIZE) {
		speed_buffer_start = (speed_buffer_start + 1) % SPEED_BUFFER_SIZE; /* full, overwrite */
		Serial.println(F("Buffer full ower writing"));
	} else {
		++speed_buffer_count;
	}
}

float bufferRead() {
	float speed = speed_buffer[speed_buffer_start];
	speed_buffer_start = (speed_buffer_start + 1) % SPEED_BUFFER_SIZE;
	--speed_buffer_count;
	return speed;
}

void sleepNow()         // here we put the arduino to sleep
{
	// Turn off the ADC while asleep.
	power_adc_disable();
	power_all_disable();

	/* Now is the time to set the sleep mode. In the Atmega8 datasheet
	 * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
	 * there is a list of sleep modes which explains which clocks and 
	 * wake up sources are available in which sleep mode.
	 *
	 * In the avr/sleep.h file, the call names of these sleep modes are to be found:
	 *
	 * The 5 different modes are:
	 *     SLEEP_MODE_IDLE         -the least power savings 
	 *     SLEEP_MODE_ADC
	 *     SLEEP_MODE_PWR_SAVE
	 *     SLEEP_MODE_STANDBY
	 *     SLEEP_MODE_PWR_DOWN     -the most power savings
	 *
	 * For now, we want as much power savings as possible, so we 
	 * choose the according 
	 * sleep mode: SLEEP_MODE_PWR_DOWN
	 * 
	 */
	set_sleep_mode (SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

	sleep_enable();          // enables the sleep bit in the mcucr register
							 // so sleep is possible. just a safety pin 

	/* Now it is time to enable an interrupt. We do it here so an 
	 * accidentally pushed interrupt button doesn't interrupt 
	 * our running program. if you want to be able to run 
	 * interrupt code besides the sleep function, place it in 
	 * setup() for example.
	 * 
	 * In the function call attachInterrupt(A, B, C)
	 * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
	 * 
	 * B   Name of a function you want to execute at interrupt for A.
	 *
	 * C   Trigger mode of the interrupt pin. can be:
	 *             LOW        a low level triggers
	 *             CHANGE     a change in level triggers
	 *             RISING     a rising edge of a level triggers
	 *             FALLING    a falling edge of a level triggers
	 *
	 * In all but the IDLE sleep modes only LOW can be used.
	 */

	attachInterrupt(PRESSURE_IRQ, wakeUpNow, RISING); // use interrupt 0 (pin 2) and run function
	// wakeUpNow when pin 2 gets LOW 

	sleep_mode();            // here the device is actually put to sleep!!
							 // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

	sleep_disable();         // first thing after waking from sleep:
							 // disable sleep...
	detachInterrupt(PRESSURE_IRQ);      // disables interrupt 0 on pin 2 so the 
	// wakeUpNow code will not be executed 
	// during normal running time.
	power_all_enable();
	Serial.print(F("Back from sleep"));

}

// Enable the CC3000 and connect to the wifi network.
// Return true if enabled and connected, false otherwise.
boolean enableConnection() {
	Serial.println(F("Turning on CC3000."));

	if (!digitalRead(ADAFRUIT_CC3000_VBAT)) {
		unsigned long t = millis();
		// Turn on the CC3000.
		wlan_start(0);
		Serial.print(F("CC3000 Started in "));
		Serial.print(millis()-t);
		Serial.println(F(" millis"));
		return false;
	}
	if (!cc3000.checkConnected()) {
		unsigned long t = millis();
		// Connect to the AP.
		if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
			// Couldn't connect for some reason.  Fail and move on so the hardware goes back to sleep and tries again later.
			Serial.println(F("Failed!"));
			return false;
		}
		Serial.print(F("Connected in "));
		Serial.print(millis()-t);
		Serial.println(F(" millis"));
		return false;
	}
	// Wait for DHCP to be complete.  Make a best effort with 5 attempts, then fail and move on.
	Serial.println(F("Request DHCP"));
//	int attempts = 0;
//	while (!cc3000.checkDHCP()) {
//		if (attempts > 5) {
//			Serial.println(F("DHCP didn't finish!"));
//			return false;
//		}
//		attempts += 1;
//		delay(100);
//	}
	if (!cc3000.checkDHCP()) {
		Serial.println(F("DHCP didn't finish!"));
		return false;
	}
	Serial.println(F("Finished DHCP"));

	if (!client.connected()) {
		if (!client.connect(mqttName)) {
			Serial.println(F("MQTT Connetion error"));
			return false;
		}
		client.publish("traffic/counter/start", mqttName);
		client.publish("traffic/counter/place", placeName);
		client.subscribe("traffic/counter/config");
		Serial.print(F("Conneted to: "));
		Serial.println(server);
	}
	// Return success, the CC3000 is enabled and connected to the network.
	return true;
}

// Disconnect from wireless network and shut down the CC3000.
void shutdownConnection() {
	if (client.connected()) {
		client.disconnect();
		Serial.println(F("Disconnected MQTT"));
	}
	// Disconnect from the AP if connected.
	// This might not be strictly necessary, but I found
	// it was sometimes difficult to quickly reconnect to
	// my AP if I shut down the CC3000 without first
	// disconnecting from the network.
	if (cc3000.checkConnected()) {
		cc3000.disconnect();
	}

	// Wait for the CC3000 to finish disconnecting before
	// continuing.
	while (cc3000.checkConnected()) {
		delay(100);
	}
	Serial.println(F("Disconnected CC3000"));

	// Shut down the CC3000.
	wlan_stop();

	Serial.println(F("CC3000 shut down."));
}

boolean connected() {
	return digitalRead(ADAFRUIT_CC3000_VBAT) && cc3000.checkConnected() && client.connected();
}

void buildTopic() {
	free(topic);
	String tmpTopic = "traffic/counter/car/";
	tmpTopic.concat(placeName);
	topic = (char*) malloc(tmpTopic.length());
	tmpTopic.toCharArray(topic, tmpTopic.length() + 1);
}

void wakeUpNow()        // here the interrupt is handled after wakeup
{
	// execute code here after wake-up before returning to the loop() function
	// timers and code using timers (serial.print and more...) will not work here.
	// we don't really need to execute any special functions here, since we
	// just want the thing to wake up
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

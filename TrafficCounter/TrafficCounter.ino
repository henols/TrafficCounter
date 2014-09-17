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
#include "PassageBuffer.h"
#include "InternetTime.h"
//#include "utility/debug.h"

#define PRESSURE_READING_PIN		A0
#define CONFIGURE_BUTTON_PIN	8
#define LED_INDICATOR_PIN		9
#define PRESSURE_IRQ			0
#define PRESSURE_IRQ_PIN		2

#define MQTT_PORT 				1883
#define PLACE_NAME_LENGTH      	32

#define PUBLISH_DELAY      		600000 // 10 min
#define CONNECTION_ALIVE_TIME	20000 // 20 sec 
#define PUBLISH_BUFFER_SIZE  	20

#define THRESHOLD       50 //change this amount if necessary. tunes sensitivity.
#define WHEEL_DELAY     50 //number of milliseconds to create accurate readings for cars. prevents bounce.
#define WHEEL_SPACEING  2.75 //average spacing between wheels of car (METERS)
#define CAR_TIMEOUT     3000
#define SPEED_BUFFER_SIZE     40

//#define INTERRUPT_FREQENCY 31250  // compare match register 16MHz/256/2Hz
#define INTERRUPT_FREQENCY 1250 // 50Hz

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

#define MQTT_SERVER 	"aceone.se"

char macHex[MAC_HEX_LENGTH];

char mqttName[MAC_HEX_LENGTH + 3];
//char placeName[PLACE_NAME_LENGTH];

//char* topic = (char*) malloc(1);
//constant char[] topic = "traffic/counter/car/DrAbrahamsVag7";
#define TOPIC_PASSAGE	"traffic/counter/car/DoktorAbrahamsVag7"

// byte mqttServer[] = { 192, 168, 1, 121 };
char mqttServer[] = MQTT_SERVER;

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS,
ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);
PubSubClient mqttClient(mqttServer, MQTT_PORT, callback, cc3000);

int triggerValue; // pressure reading THRESHOLD for identifying a bike is pressing.
int max = 0;
int isMeasuring = 0;
int countThis = 0;
int strikeNumber = 0;
float firstWheel = 0.0;
float secondWheel = 0.0;
float wheelTime = 0.0;
float speed = 0.0;

PassageBuffer passageBuffer(SPEED_BUFFER_SIZE);
Passage passage;

volatile unsigned long timeStamp;
volatile unsigned long wakeUpTime;

int initError = 1;

void setup() {

	Serial.begin(115200);

	pinMode(PRESSURE_READING_PIN, INPUT);
	//	pinMode(CONFIGURE_BUTTON_PIN, INPUT_PULLUP);
	pinMode(LED_INDICATOR_PIN, OUTPUT);
	pinMode(PRESSURE_IRQ_PIN, INPUT);

//	Serial.print("Free RAM: ");
//	Serial.println(getFreeRam(), DEC);
	Serial.print(F("Init"));

	digitalWrite(LED_INDICATOR_PIN, HIGH);

	if (!cc3000.begin()) {
		errorMessage(2);
	}

	if (!cc3000.deleteProfiles()) {
		errorMessage(3);
	}

	uint8_t macAddress[MAC_LENGTH];

	cc3000.getMacAddress(macAddress);
	bytesToHexString(macAddress, macHex, MAC_LENGTH);

	String tmpName = "TC-";
	tmpName.concat(macHex);
	mqttName[tmpName.length()];
	tmpName.toCharArray(mqttName, tmpName.length() + 1);

	// read local air pressure and create offset.
	triggerValue = analogRead(PRESSURE_READING_PIN) + THRESHOLD;
	Serial.println(F(" OK"));

	delay(1000);
	Serial.println(F("Traffic Counter"));
	Serial.println();
//	Serial.println();
//	Serial.print(F("Place: "));
//	Serial.println(placeName);
	Serial.print(F("Air Pressure: "));
	Serial.println(triggerValue - THRESHOLD);
	Serial.print(F("Mqtt name: "));
	Serial.println(mqttName);
	Serial.println();

	while (!enableConnection()) {
		shutdownConnection();
		delay(1000);
	}
	digitalWrite(LED_INDICATOR_PIN, LOW);

	initError = 0;
	startTimeIntrrupt();

//	delay(1000);
//  Serial.println(F("Shut down!"));
//	shutdownConnection();
//	delay(1000);
//  Serial.println(F("Sleep now!"));
//	Serial.print("Free RAM: ");
//	Serial.println(getFreeRam(), DEC);
//	delay(100);
//	sleepNow();

	timeStamp = millis();
}

void loop() {
	mqttClient.loop();

	unsigned long ml = millis();

	if (ml > timeStamp + CONNECTION_ALIVE_TIME && connected()) {
		shutdownConnection();
		delay(100);
		sleepNow();
	}
	if (passageBuffer.nrElem() > PUBLISH_BUFFER_SIZE || ml > timeStamp + PUBLISH_DELAY) {
		if (connected()) {
			publishBuffer();
		} else {
			if (!enableConnection()) {
				shutdownConnection();
				timeStamp += PUBLISH_DELAY;
			}
		}
	}
}

void checkPressure() {
//	digitalWrite(LED_INDICATOR_PIN, !digitalRead(LED_INDICATOR_PIN));
	int analogVal = analogRead(PRESSURE_READING_PIN);
	//1 - TUBE IS PRESSURIZED INITIALLY
	if (analogVal > triggerValue) {
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
	while (analogVal > max && isMeasuring == 1) { //is being pressed, in all cases. to measure the max pressure.
		max = analogVal;
	}

	//3 - TUBE IS RELEASED
	if (analogVal < triggerValue - 1 && countThis == 0) { //released by either wheel
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
	if ((analogVal < triggerValue - 1)
			&& ((countThis == 1 && isMeasuring == 0)
					|| ((millis() - firstWheel) > CAR_TIMEOUT)
							&& isMeasuring == 1)) { //has been released for enough time.
//		Serial.print(F("Pressure Reached = "));
//		Serial.println(max);
//		Serial.print(F("time between wheels = "));
		wheelTime = ((secondWheel - firstWheel) / 3600000);
//		Serial.println(wheelTime);
		int time = ((millis() / 1000) / 60) + 1; // the number of seconds since first record.
		speed = (WHEEL_SPACEING / 1000) / wheelTime;
		if (speed > 0) {
			Serial.print(F("Estimated Speed (km/h) = "));
			Serial.println(speed);
		} else {
			speed = 1.0;
		}

		passage.speed = speed;
		if (it_appTime()) {
			passage.timeStamp = it_currentSeconds();
		} else {
			passage.timeStamp = (millis() - wakeUpTime) / 1000;
		}
		publish(passage);

		//RESET ALL VALUES
		max = 0;
		strikeNumber = 0;
		countThis = 0;
		isMeasuring = 0;
	}
}

void publishBuffer() {
	while (!passageBuffer.isEmpty()) {
		passageBuffer.read(&passage);
		unsigned long timeDiff = (it_appTime() - wakeUpTime) / 1000;
		if (passage.timeStamp < it_appTime() / 1000) {
			passage.timeStamp = it_currentSeconds() - timeDiff
					+ (passage.timeStamp);
		}
		publishToClient(passage);
		timeStamp = millis();
	}
}

void publish(Passage passage) {
	if (passageBuffer.isEmpty() && connected()) {
		publishToClient(passage);
		timeStamp = millis();
	} else {
		passageBuffer.write(&passage);
	}
}

void publishToClient(Passage passage) {
	char tmp[11];
	char msg[20];
	dtostrf(passage.speed, 1, 2, tmp);
	strcpy(msg, tmp);
	strcat(msg, ":");
	ultoa(passage.timeStamp, tmp, 10);
	strcat(msg, tmp);
	strcat(msg, "000");
//	mqttClient.publish(topic, msg);
	mqttClient.publish(TOPIC_PASSAGE, msg);
//	Serial.println(msg);
}

void sleepNow()         // here we put the arduino to sleep
{
	it_clearAppTime();

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
//  Serial.print(F("Back from sleep"));

}

void startTimeIntrrupt() {
	// initialize timer1 
	noInterrupts();           // disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;

	OCR1A = INTERRUPT_FREQENCY;
	TCCR1B |= (1 << WGM12);   // CTC mode
	TCCR1B |= (1 << CS12);    // 256 prescaler 
	TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
	interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect)// timer compare interrupt service routine
{
	checkPressure();
}

// Enable the CC3000 and connect to the wifi network.
// Return true if enabled and connected, false otherwise.
boolean enableConnection() {
	Serial.print(F("Conneting to WIFI "));

	if (!digitalRead(ADAFRUIT_CC3000_VBAT)) {
		// Turn on the CC3000.
		wlan_start(0);
	}
	if (!cc3000.checkConnected()) {
		// Connect to the AP.
		if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
			// Couldn't connect for some reason.  Fail and move on so the hardware goes back to sleep and tries again later.
			if (initError) {
				errorMessage(4);
			}
			return false;
		}
	}

	int attempts = 0;
	while (!cc3000.checkDHCP()) {
		if (attempts > 50) {
			return false;
		}
		attempts += 1;
		delay(200);
	}

	//	while (!cc3000.checkDHCP()) {
	//		delay(100);
	//	}
//	Serial.println(F("Finished DHCP"));

	if (!mqttClient.connected()) {
		if (!mqttClient.connect(mqttName)) {
			if (initError) {
				errorMessage(6);
			}
			return false;
		}
		mqttClient.publish("traffic/counter/start", mqttName);
//		mqttClient.publish("traffic/counter/place", placeName);
//		mqttClient.subscribe("traffic/counter/config");
//		Serial.print(F("Conneted to: "));
//		Serial.print(mqttServer);
//		Serial.println();
	}

	// Return success, the CC3000 is enabled and connected to the network.
	timeStamp = millis();
	if (!it_getTime(&cc3000)) {
		Serial.print(F(" no time "));
	}
	Serial.println(F("OK"));
	return true;
}

// Disconnect from wireless network and shut down the CC3000.
void shutdownConnection() {
	if (mqttClient.connected()) {
		mqttClient.publish("traffic/counter/stop", mqttName);
		mqttClient.disconnect();
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
//  Serial.println(F("Disconnected CC3000"));

	// Shut down the CC3000.
	wlan_stop();

	Serial.println(F("CC3000 shut down."));
}

boolean connected() {
	return digitalRead(ADAFRUIT_CC3000_VBAT) && cc3000.checkConnected()
			&& mqttClient.connected();
}


void wakeUpNow()        // here the interrupt is handled after wakeup
{
	power_all_enable();
	wakeUpTime = millis();
}

void callback(char* topic, byte* payload, unsigned int length) {
//	// handle message arrived
//	if (length < MAC_HEX_LENGTH) {
//		return;
//	}
//
//	for (int i = 0; i < MAC_HEX_LENGTH - 1; i++) {
//		if (payload[i] != macHex[i]) {
//			return;
//		}
//	}
//
//	int len = length - MAC_HEX_LENGTH + 1;
//	// Allocate the correct amount of memory for the payload copy
//	char* p = (char*) malloc(len);
//	int i;
//	// Copy the payload to the new buffer
//	for (i = 0; i < len; i++) {
//		p[i] = payload[MAC_HEX_LENGTH + i];
//	}
//	p[i] = '\0';
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

void errorMessage(int nr) {
	while (1) {
		for (int i = 0; i < nr; i++) {
			digitalWrite(LED_INDICATOR_PIN, HIGH);
			delay(200);
			digitalWrite(LED_INDICATOR_PIN, LOW);
			delay(200);
		}
		delay(1000);
	}
}


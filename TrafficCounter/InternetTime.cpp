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

// ensure this library description is only included once

#include "InternetTime.h"

char* PROGMEM ntpPool[] = POOL_NTP_ORG;

volatile unsigned long lastPolledTime = 0L; // Last value retrieved from time server
volatile unsigned long appTime = 0L; // CPU milliseconds since last server query

// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long it_getTime(Adafruit_CC3000* cc3000) {

	uint8_t buf[48];
	unsigned long ip, startTime, t = 0L;
	Adafruit_CC3000_Client client;

//  Serial.print(F("Locating time server..."));
	for (int i; i < sizeof(ntpPool); i++) {
		// Hostname to IP lookup; use NTP pool (rotates through servers)
		if (cc3000->getHostByName(ntpPool[i], &ip)) {
			static const char PROGMEM
			timeReqA[] = {227, 0, 6, 236},
			timeReqB[] = {49, 78, 49, 52};

//    Serial.println(F("\r\nAttempting connection..."));
			startTime = millis();
			do {
				client = cc3000->connectUDP(ip, 123);
			} while ((!client.connected())
					&& ((millis() - startTime) < connectTimeout));

			if (client.connected()) {
//      Serial.print(F("connected!\r\nIssuing request..."));

				// Assemble and issue request packet
				memset(buf, 0, sizeof(buf));
				memcpy_P(buf, timeReqA, sizeof(timeReqA));
				memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
				client.write(buf, sizeof(buf));

//      Serial.print(F("\r\nAwaiting response..."));
				memset(buf, 0, sizeof(buf));
				startTime = millis();
				while ((!client.available())
						&& ((millis() - startTime) < responseTimeout))
					;
				if (client.available()) {
					client.read(buf, sizeof(buf));
					t = ((((unsigned long) buf[40] << 24)
							| ((unsigned long) buf[41] << 16)
							| ((unsigned long) buf[42] << 8)
							| (unsigned long) buf[43]) - 2208988800UL);
//        Serial.print(F("OK\r\n"));
				}
				client.close();
			}
		}
		if (t) {
			lastPolledTime = t;         // Save time
			appTime = millis();  // Save sketch time of last valid time query
			return 1;
		}
	}
	return 0;
}

unsigned long it_currentSeconds() {
	return lastPolledTime + (millis() - appTime) / 1000;
}

unsigned long it_appTime() {
	return appTime;
}

void it_clearAppTime() {
	appTime = 0;
}


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
#include <ccspi.h>
#include <SPI.h>
#include <PubSubClient.h>

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed but DI

#define WLAN_SSID       "myNetwork"        // cannot be longer than 32 characters!
#define WLAN_PASS       "myPassword"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

byte server[] = { 172, 16, 0, 2 };

PubSubClient client(server, 1883, callback, cc3000);

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

#define THRESHOLD       2 //change this amount if necessary. tunes sensitivity.
#define WHEEL_DELAY     50 //number of milliseconds to create accurate readings for cars. prevents bounce.
#define WHEEL_SPACEING  2.7500 //average spacing between wheels of car (METERS)
#define CAR_TIMEOUT     3000

int triggerValue; // pressure reading THRESHOLD for identifying a bike is pressing.
int the_tally; //total amount of sensings.
int max = 0;
int isMeasuring = 0;
int countThis = 0;
int strikeNumber = 0;
float firstWheel = 0.0000000;
float secondWheel= 0.0000000;
float wheelTime = 0.0000000;
float speed = 0.0000000;


void setup() {
  pinMode(A0, INPUT);
  pinMode(2, OUTPUT);
  pinMode(13, OUTPUT);

  Serial.begin(115200);

  Serial.print("Conecting to WIFI .");

  cc3000.begin();

  Serial.print(".");

  if (!cc3000.deleteProfiles()) {
    while(1);
  }

  Serial.print(".");
  char *ssid = WLAN_SSID;             /* Max 32 chars */
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    while(1);
  }
   
  Serial.print(".");
  while (!cc3000.checkDHCP())
  {
    Serial.print(".");
    delay(100); // ToDo: Insert a DHCP timeout!
  }
    Serial.println(" OK");

  if (client.connect("arduinoClient")) {
    client.publish("outTopic","hello world");
    client.subscribe("inTopic");
  }

  // read local air pressure and create offset.
  triggerValue = analogRead(A0) + THRESHOLD;
  delay(1000);
  Serial.println("Hello, Welcome to the DIY Traffic Counter");
  Serial.println("Developed by Tomorrow Lab in NYC");
  Serial.println("___________________________________________________");
  Serial.println("");
  Serial.print("Local Air Pressure: ");
  Serial.println(triggerValue - THRESHOLD);
  Serial.println("___________________________________________________");


}

void loop() {
  //Serial.println(analogRead(A0));

  //1 - TUBE IS PRESSURIZED INITIALLY
  if (analogRead(A0) > triggerValue) {
    if (strikeNumber == 0 && isMeasuring == 0) { // FIRST HIT
      Serial.println("");
      Serial.println("Car HERE. ");
      firstWheel = millis(); 
      isMeasuring = 1;
    }
    if (strikeNumber == 1 && isMeasuring == 1) { // SECOND HIT
      Serial.println("Car GONE.");
      secondWheel = millis();
      isMeasuring = 0;
    }
  }


  //2 - TUBE IS STILL PRESSURIZED
  while(analogRead(A0) > max && isMeasuring == 1) { //is being pressed, in all cases. to measure the max pressure.
    max = analogRead(A0); 
  }


  //3 - TUBE IS RELEASED
  if (analogRead(A0) < triggerValue - 1 && countThis == 0) { //released by either wheel
    if (strikeNumber == 0 && isMeasuring == 1 && (millis() - firstWheel > WHEEL_DELAY)) {
      strikeNumber = 1;
    }
    if (strikeNumber == 1 && isMeasuring == 0 && (millis() - secondWheel > WHEEL_DELAY) ) {
      countThis = 1;
    }
  }


  //4 - PRESSURE READING IS ACCEPTED AND RECORDED
  if ((analogRead(A0) < triggerValue - 1) && ((countThis == 1 && isMeasuring == 0) || ((millis() - firstWheel) > CAR_TIMEOUT) && isMeasuring == 1)) { //has been released for enough time.
    the_tally++; 
    Serial.print("Pressure Reached = ");
    Serial.println(max);
    Serial.print("Current Count = ");
    Serial.println(the_tally);
    //Serial.print("time between wheels = ");
    wheelTime = ((secondWheel - firstWheel)/3600000);
    //Serial.println(wheelTime);
    int time = ((millis()/1000)/60) + 1; // the number of seconds since first record.
    speed = (WHEEL_SPACEING/1000)/wheelTime;
    if (speed > 0 ) {
      Serial.print("Estimated Speed (km/h) = ");
      Serial.println(speed);
      char tmp[10];
       dtostrf(speed,1,2,tmp);
      client.publish("trafffic/count/car",tmp);
    }
    else {
      client.publish("trafffic/count/car","-1");
    }

    //RESET ALL VALUES
    max = 0; 
    strikeNumber = 0;
    countThis = 0;
    isMeasuring = 0;

  }
}





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
#ifndef InternetTime_h
#define InternetTime_h

#include "Arduino.h"
#include <Adafruit_CC3000.h>

#define POOL_NTP_ORG	{"0.se.pool.ntp.org", "1.se.pool.ntp.org", "2.se.pool.ntp.org", "3.se.pool.ntp.org"}

const unsigned long connectTimeout = 15L * 1000L; // Max time to wait for server connection
const unsigned long responseTimeout = 15L * 1000L; // Max time to wait for data from server


// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long it_getTime(Adafruit_CC3000* cc3000);
unsigned long it_currentSeconds();
unsigned long it_appTime();
void it_clearAppTime();

#endif

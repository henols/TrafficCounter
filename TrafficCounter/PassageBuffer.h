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
#ifndef PassageBuffer_h
#define PassageBuffer_h

#include "Arduino.h"

struct Passage {
    float speed;
    unsigned long timeStamp;
} ;

// library interface description
class PassageBuffer
{
  // user-accessible "public" interface
  public:
	PassageBuffer(int);
	void freeBuf();
 
	int isFull();
 
	int isEmpty();

	int nrElem();
 
/* Write an element, overwriting oldest element if buffer is full. App can
   choose to avoid the overwrite by checking cbIsFull(). */
void write(Passage *elem);

/* Read oldest element. App must ensure !cbIsEmpty() first. */
void read(Passage *elem);

  // library-accessible "private" interface
  private:
    int _size;
int     start;
int     end;
    Passage *elems;

};

#endif


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

// include this library's description file
#include "PassageBuffer.h"


PassageBuffer::PassageBuffer(int _size) {
	this->_size = _size + 1; /* include empty elem */
	start = 0;
	end = 0;
	elems = (Passage*) calloc(_size, sizeof(Passage));
}


void PassageBuffer::freeBuf() {
	free(elems); /* OK if null */
}

int PassageBuffer::isFull() {
	return (end + 1) % _size == start;
}

int PassageBuffer::nrElem() {
	return (_size + end - start) % _size;
}

int PassageBuffer::isEmpty() {
	return end == start;
}

/* Write an element, overwriting oldest element if buffer is full. App can
 choose to avoid the overwrite by checking cbIsFull(). */
void PassageBuffer::write(Passage *elem) {
	elems[end] = *elem;
	end = (end + 1) % _size;
	if (end == start)
		start = (start + 1) % _size; /* full, overwrite */
}

/* Read oldest element. App must ensure !cbIsEmpty() first. */
void PassageBuffer::read(Passage *elem) {
	*elem = elems[start];
	start = (start + 1) % _size;
}


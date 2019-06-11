/*
The MIT License (MIT)

Copyright (c) 2018 WHILL,

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "WHILL.h"
#include <string.h>

WHILL::Packet::Packet(){
  protocol_sign = 0xAF;
  len = 1;
  build();
}

WHILL::Packet::Packet(unsigned char payload[],int size){
  protocol_sign = 0xAF;
  len = size + 1;
  memcpy(this->payload,payload,size);
  build();
}

bool WHILL::Packet::is_valid(){
  return getCalculatedCS() == cs;
}

int WHILL::Packet::rawLength(){
  return 2 + len;   // protocol_sign + len + (the length of payload and cs)
}

bool WHILL::Packet::setRaw(unsigned char* raw,int whole_length){

    protocol_sign = raw[0];
    len           = raw[1];

    if(len >= MAX_LENGTH) return false; // Is not valid

    int prefix = 2;
    int i = 0;
    for(i=0;i<len-1;i++){
      payload[i]= raw[prefix+i];
    }

    cs = raw[prefix+i];
    
    return is_valid();
}

unsigned char WHILL::Packet::getCalculatedCS(){
  unsigned char cs = 0x00;

  cs ^= protocol_sign;
  cs ^= len;

  for(int i=0;i<len-1;i++){
    cs ^= payload[i];
  }

  return cs;
}

void WHILL::Packet::build(){
  this->cs = getCalculatedCS();
}

int WHILL::Packet::getRaw(unsigned char* raw){

  int whole_length = 0;

  raw[0] = protocol_sign;
  raw[1] = len;

  int prefix = 2;
  int i = 0;
  for(i=0;i<len-1;i++){
    raw[prefix+i] = payload[i];
  }

  raw[prefix+i] = cs;
  
  return rawLength();
}


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

int WHILL::PacketReceiver::push(unsigned char data){

    if(!recording){
        if(data != WHILL::Packet::PROTOCOL_SIGN){
            index = 0;
            return -1;
        }else{
            recording = true;
        }
    }

    if (index > Packet::MAX_LENGTH){
        // Prevent Buffer Over Run
        index = 0;
        recording = false;
        return -1;
    }

    buf[index] = data;

    if(index >= 1 && remaining_bytes() == 0){
        call_callback();
        index = 0;
        recording = false;
        return 0;
    }

    index++;
    return remaining_bytes();
}

int WHILL::PacketReceiver::remaining_bytes(){
    if(index == 0)return -1;
    if(!recording)return -1;

    int length = 2 + buf[1];  // Protocl sign + length + [len](payload + cs)

    return length-(index+1);
}

void WHILL::PacketReceiver::register_callback(void (*callback)()){
    this->obj    = NULL;
    this->method = NULL;
    this->callback = callback;
}

void WHILL::PacketReceiver::register_callback(PacketParser* obj,int(PacketParser::*method)(WHILL::Packet* packet)){
    this->obj = obj;
    this->method = method;
    this->callback = NULL;
}



bool WHILL::PacketReceiver::call_callback(){

    Packet packet;

    if(packet.setRaw(buf,index+1) == false){ // If packet was broken 
        return false;
    }


    if(callback != NULL){
        this->callback();
        return true;
    }

    if(obj != NULL && method != NULL){
        (obj->*method)(&packet);
        return true;
    }

    return false;
}

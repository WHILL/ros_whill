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

WHILL::WHILL(int (*read)(std::vector<uint8_t> &data), int (*write)(std::vector<uint8_t> &data))
{
    this->read = read;
    this->write = write;

    parser.setParent(this);
    receiver.register_callback(&parser,&PacketParser::parsePacket);
}


// int WHILL::read(unsigned char* byte){    // Implementation of read interaface to WHILL
//     if(serial == NULL) return -1;

//     int data = serial->read();
//     if(data == -1) return -1;  //Nothing read

//     *byte = data;

//     return 1;
// }

// int WHILL::write(unsigned char byte){   // Implementation of write interface to WHILL
//     if(serial == NULL) return -1;
//     serial->write(byte);

//     return 1;
// }

void WHILL::begin(unsigned int interval){
    this->startSendingData1(interval);
}

void WHILL::transferPacket(Packet* packet){
    unsigned char buffer[Packet::MAX_LENGTH] = {0};
    int size = packet->getRaw(buffer);
    std::vector<uint8_t>data(buffer, buffer + size * sizeof(buffer[0]));
    write(data);
}

void WHILL::receivePacket(){
    std::vector<uint8_t> data;

    if(read(data) > 0){
        size_t size = data.size();
        for (size_t i = 0; i < size;i++){
            receiver.push(data[i]);
        }
    }

}

// void WHILL::keep_joy_delay(unsigned long ms){
//     while(ms > 0){
//         refresh();
//         if(ms%100 == 0){
//             this->setJoystick(virtual_joy.x,virtual_joy.y);
//         }
//         ms--;
//         ::delay(1);
//     }  
// }


// void WHILL::delay(unsigned long ms){
//     while(ms > 0){
//         refresh();
//         ms--;
//         ::delay(1);
//     }
// }


void WHILL::refresh(){
    // Scan the data from interface
   receivePacket();
}


void WHILL::register_callback(Callback method,EVENT event){
    callback_functions[event] = method;
}

void WHILL::fire_callback(EVENT event){
    if(callback_functions[event]==NULL)return;
    callback_functions[event](this);
}
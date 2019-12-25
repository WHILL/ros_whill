/*
The MIT License (MIT)

Copyright (c) 2018-2019 WHILL,

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
#include <stdio.h>

void WHILL::startSendingData0(unsigned int interval_ms,unsigned char speed_mode){
    unsigned char payload[] =  {0x00,  //Start Sending Data
                                0x00,  //Data0 (Speed profiles)
                                (unsigned char)(interval_ms<<8 & 0xFF),
                                (unsigned char)(interval_ms<<0 & 0xFF),
                             speed_mode};
    Packet packet(payload,sizeof(payload));
    packet.build();
    transferPacket(&packet);
}

void WHILL::startSendingData1(unsigned int interval_ms){
    unsigned char payload[] =  {0x00,   //Start Sending Data
                                0x01,   //Data1  (Sensors)
                                (unsigned char)(interval_ms<<8 & 0xFF),
                                (unsigned char)(interval_ms<<0 & 0xFF),
                                0x00};
    Packet packet(payload,sizeof(payload));
    packet.build();
    transferPacket(&packet);
}

void WHILL::stopSendingData(){
    unsigned char payload[] =  {0x01};  // Stop Sending Data
    Packet packet(payload,sizeof(payload));
    packet.build();
    transferPacket(&packet);
}

void WHILL::setPower(bool power){

    unsigned char payload[] = {0x02,
                               (unsigned char)(power ? 0x01 : 0x00)};
    Packet packet(payload,sizeof(payload));
    packet.build();
    if(power){
        transferPacket(&packet);
        sleep_ms(10);
    }
    transferPacket(&packet);
}

void WHILL::setJoystick(int x,int y){
    virtual_joy.x = x;
    virtual_joy.y = y;

    unsigned char payload[] = {0x03,
                               0x00,   // Enable Host control
                               (unsigned char)(char)(y),
                               (unsigned char)(char)(x)};
    Packet packet(payload,sizeof(payload));
    packet.build();
    transferPacket(&packet);
}


bool WHILL::setSpeedProfile(SpeedProfile &profile,unsigned char speed_mode){

    if(profile.check() != SpeedProfile::Error::NoError)
        return false;

    unsigned char payload[] = {0x04,
                               speed_mode,
                               profile.forward.speed,
                               profile.forward.acc,
                               profile.forward.dec,
                               profile.backward.speed,
                               profile.backward.acc,
                               profile.backward.dec,
                               profile.turn.speed,
                               profile.turn.acc,
                               profile.turn.dec};                         
    Packet packet(payload,sizeof(payload));
    packet.build();
    transferPacket(&packet);
    return true;
}

void WHILL::setBatteryVoltaegeOut(bool enable){
     unsigned char payload[] = {0x05,
                                (unsigned char)(enable ? 0x01 : 0x00)};
    Packet packet(payload,sizeof(payload));
    packet.build();
    transferPacket(&packet);
}

// Experimental, Speed control without Jerk control but only Acceleration cotnrol.
void WHILL::setSpeed(float linear,  float angular)
{
    int16_t x = 0, y = 0; // Unit: 0.004km/h

    // Linear
    y = (linear * 3.6f) / 0.004f;

    // angular
    const float tread_width = 0.248; // [m]
    float v = tread_width * angular;  // V=rω [m/s]
    x = -v * 3.6f / 0.004f * 2.0f; // [m/s] to [km/h]→ 0.004km/h unit, Double for differencial

    unsigned char payload[] = {
        0x08, // Experimental Command, Control with Low Jerk, Almost Const-Accel control
        0,    // 0=Disable Joystick 1=Allow Joystick
        (uint8_t)((y >> 8) & 0xff),
        (uint8_t)((y >> 0) & 0xff),
        (uint8_t)((x >> 8) & 0xff),
        (uint8_t)((x >> 0) & 0xff)};
    Packet packet(payload, sizeof(payload));
    packet.build();
    transferPacket(&packet);
}

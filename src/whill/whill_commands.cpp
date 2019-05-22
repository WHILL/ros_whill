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

    std::vector<uint8_t> nop = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

    unsigned char payload[] = {0x02,
                               (unsigned char)(power ? 0x01 : 0x00)};
    Packet packet(payload,sizeof(payload));
    packet.build();
    if(power){
        write(nop);
        sleep_ms(5);
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
    int x, y;

    const float front_max = 6.0;     // [km/h]
    const float spin_max = 3.5;      // [km/h]
    const float back_max = 2.0;      // [km/h]
    const float tread_width = 0.248; // [m]

    float desire_front_kmh = linear * 3.6;                    // [m/s]   to [km/h]
    float desire_spin_spd = -tread_width * angular * 3.6 * 2; // [rad/s] to [km/h]

    if (linear >= 0)
    {
        //forward
        y = desire_front_kmh / front_max * 100;
        y = y > 100 ? 100 : y;
    }
    else
    {
        //back
        y = desire_front_kmh / back_max * 100;
        y = y < -100 ? -100 : y;
    }

    x = desire_spin_spd / spin_max * 100;
    x = x > 100 ? 100 : x;
    x = x < -100 ? -100 : x;

    unsigned char payload[] = {0x08, // Experimental Command, Control with Low Jerk, Almost Const-Accel control
                               0x00, 
                               (unsigned char)(char)(y),
                               (unsigned char)(char)(x)};
    Packet packet(payload, sizeof(payload));
    packet.build();
    transferPacket(&packet);
}


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

#ifndef __WHILL_H__
#define __WHILL_H__

#include <stdint.h>
#include <vector>

class WHILL
{

    class Packet
    {

    private:
    public:
        const static unsigned char PROTOCOL_SIGN = 0xAF;

        const static int MAX_LENGTH = 35;
        const static int MAX_PAYLOAD = MAX_LENGTH - 3; // protocol_sign,len,cs

        const static int DATASET1_LEN_V01 = 32; // Stable
        const static int DATASET1_LEN_V02 = 33; // Experimental, Additional timing data from Motor Controller

        Packet();
        Packet(unsigned char payload[], int size);

        unsigned char getCalculatedCS();

        unsigned char protocol_sign;
        unsigned char len;
        unsigned char payload[MAX_LENGTH];
        unsigned char cs;

        bool is_valid();
        int rawLength();

        bool setRaw(unsigned char *raw, int len);
        int getRaw(unsigned char *raw);
        void build();
    };

    class PacketParser
    {

    private:
        WHILL *whill = NULL;
        void parseDataset0(WHILL::Packet *packet);
        void parseDataset1(WHILL::Packet *packet);

    public:
        void setParent(WHILL *whill);
        void setWHILLReceiver(WHILL *whill);
        int parsePacket(Packet *packet);
    };

    class PacketReceiver
    {

    private:
        unsigned char buf[Packet::MAX_LENGTH] = {0};
        unsigned char index = 0;
        bool recording = false;

        void (*callback)() = nullptr;
        bool call_callback();

        PacketParser *obj = nullptr;
        int (PacketParser::*method)(WHILL::Packet *packet) = nullptr;

    public:
        int push(unsigned char data);
        int remaining_bytes();
        void reset();
        void register_callback(void (*callback)());
        void register_callback(PacketParser *obj, int (PacketParser::*method)(WHILL::Packet *packet));
    };

private:
    // Custom transceiver
    int (*read)(std::vector<uint8_t> &data);  // Returns how many bytes read actually
    int (*write)(std::vector<uint8_t> &data); // Returns how many bytes wrote actually
    void (*sleep_ms)(uint32_t ms);                      // Sleep function (ms)

    void receivePacket();
    void transferPacket(Packet *packet);

    PacketReceiver receiver;
    PacketParser parser;

    // Experimantal
    int16_t past_time_ms = -1;  // not received any data yet if negative
    static uint8_t calc_time_diff(uint8_t past, uint8_t current);

public:
    WHILL(int (*read)(std::vector<uint8_t> &data), int (*write)(std::vector<uint8_t> &data), void (*sleep)(uint32_t ms));
    void begin(uint8_t interval);

    const float wheel_radius = 0.1325;
    const float tread = 0.496;

    //Callback
    enum EVENT
    {
        CALLBACK_DATA0,
        CALLBACK_DATA1,
        CALLBACK_POWER_ON,
        EVENT_SIZE
    };
    typedef void (*Callback)(WHILL *);
    Callback callback_functions[EVENT_SIZE] = {NULL};
    void register_callback(Callback method, EVENT event);
    void fire_callback(EVENT event);

    void refresh();

    class SpeedProfile
    {
    private:
        template <typename U,typename T,typename Z>
        bool checkRange(U min, T value, Z max);

    public:
        enum Error
        {
            NoError,
            InvalidForwardSpeed,
            InvalidBackwardSpeed,
            InvalidTurnSpeed,
            InvalidForwardAcc,
            InvalidBackwardAcc,
            InvalidTurnAcc,
            InvalidForwardDec,
            InvalidBackwardDec,
            InvalidTurnDec,
        };

        class Pack
        {
            public:
            uint8_t speed;
            uint8_t acc;
            uint8_t dec;
        };
        
        Error check();
        Pack forward;
        Pack backward;
        Pack turn;
    };

    typedef struct
    {
        int x;
        int y;
    } Joy;

    typedef struct
    {
        unsigned char level;
        signed long current;
    } Battery;

    typedef struct
    {
        float angle;
        int speed;
    } Motor;

    Joy virtual_joy = {0};
    Joy joy = {0};
    Battery battery = {0};
    Motor left_motor = {0};
    Motor right_motor = {0};
    bool power = false;
    int speed_mode_indicator = -1;

    // Experimental
    int32_t _interval = -1; // ms

    //WHILL commands
    void startSendingData0(unsigned int interval_ms, unsigned char speed_mode);
    void startSendingData1(unsigned int interval_ms);
    void stopSendingData();
    void setJoystick(int x, int y);
    void setPower(bool power);
    void setBatteryVoltaegeOut(bool out);
    bool setSpeedProfile(SpeedProfile &profile, unsigned char speed_mode);

    // Experimental
    void setSpeed(float linear, float angular); // Linear:[m/s], Angular:[rad/s]
};

#endif
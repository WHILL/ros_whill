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

#include <stdio.h>


class WHILL{

    class Packet{

        private:

        public:
            const static unsigned char PROTOCOL_SIGN = 0xAF;

            const static int MAX_LENGTH   = 35;
            const static int MAX_PAYLOAD  = MAX_LENGTH - 3;  // protocol_sign,len,cs

            Packet();
            Packet(unsigned char payload[],int size);

            unsigned char       getCalculatedCS();

            unsigned char       protocol_sign;
            unsigned char       len;
            unsigned char       payload[MAX_LENGTH];
            unsigned char       cs;

            bool is_valid();

            int  rawLength();

            bool setRaw(unsigned char* raw,int len);
            int  getRaw(unsigned char* raw);
            void build();
    };



    class PacketParser{

        private:
            WHILL* whill = NULL;
            void parseDataset0(WHILL::Packet* packet);
            void parseDataset1(WHILL::Packet* packet);

        public:
            void setParent(WHILL* whill);
            void setWHILLReceiver(WHILL* whill);
            int parsePacket(Packet* packet); 

    };


    class PacketReceiver{

        private:
            unsigned char buf[Packet::MAX_LENGTH]   = {0};
            unsigned char index     = 0;
            bool          recording = false;

            void(*callback)()       = NULL;
            bool          call_callback();

            PacketParser* obj       = NULL;
            int (PacketParser::*method)(WHILL::Packet* packet) = NULL;

        public:
            int push(unsigned char data);
            int remaining_bytes();
            void reset();
            void register_callback(void (*callback)());
            void register_callback(PacketParser* obj,int(PacketParser::*method)(WHILL::Packet* packet));
    };



private:

    // Custom transceiver
    int (*read)(unsigned char *byte);
    int (*write)(unsigned char bytes[],int length);

    void receivePacket();
    void transferPacket(Packet* packet);

    PacketReceiver receiver;
    PacketParser   parser;


public:
  WHILL(int (*read)(unsigned char *byte), int (*write)(unsigned char bytes[], int length));
  void begin(unsigned int interval);

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

  void keep_joy_delay(unsigned long ms);
  void delay(unsigned long ms);

  typedef struct
  {
      unsigned char forward_spped;
      unsigned char forward_acceleration;
      unsigned char forward_deceleration;

      unsigned char reverse_speed;
      unsigned char reverse_acceleration;
      unsigned char reverse_deceleration;

      unsigned char turn_speed;
      unsigned char turn_acceleration;
      unsigned char turn_deceleration;
      } SpeedProfile;

      typedef struct
      {
          int x;
          int y;
          int z;
      } Data3D;

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

      Data3D accelerometer = {0};
      Data3D gyro = {0};
      Joy virtual_joy = {0};
      Joy joy = {0};
      Battery battery = {0};
      Motor left_motor = {0};
      Motor right_motor = {0};
      bool power = false;
      int speed_mode_indicator = -1;

      //WHILL commands
      void startSendingData0(unsigned int interval_ms, unsigned char speed_mode);
      void startSendingData1(unsigned int interval_ms);
      void stopSendingData();
      void setJoystick(int x, int y);
      void setPower(bool power);
      void setBatteryVoltaegeOut(bool out);
      void setSpeedProfile(SpeedProfile * profile, unsigned char speed_mode);

};


#endif
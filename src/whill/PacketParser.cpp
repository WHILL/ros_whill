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
#include "stdio.h"

#include "WHILL.h"

void WHILL::PacketParser::setParent(WHILL* whill){
  this->whill = whill;
}

int WHILL::PacketParser::parsePacket(Packet* packet){
  if(!(packet->is_valid()))return -1;

  switch(packet->payload[0]){  // Read Command ID

      case 0x00:   // Data set 0
        parseDataset0(packet);
        break;

      case 0x01:   // Data set 1
        parseDataset1(packet);
        break;

      case 0x52:   // Response of power WHILL on.
        if(whill!=NULL){
          whill->fire_callback(CALLBACK_POWER_ON);
        }
        break;

      default:
        return -1;  // Unknown Command
  }

  return 0;

}


void WHILL::PacketParser::parseDataset0(Packet* packet){
  if(whill == NULL) return;
  whill->fire_callback(CALLBACK_DATA0);
}

void WHILL::PacketParser::parseDataset1(Packet* packet){
  whill->joy.y = (int)(signed char)packet->payload[13];
  whill->joy.x = (int)(signed char)packet->payload[14];

  whill->battery.level   = (unsigned char)packet->payload[15];
  whill->battery.current = (signed short)((packet->payload[16] << 8)+(packet->payload[17])) * 2; //Unit : 2mA

  whill->right_motor.angle = (float)((signed short)((packet->payload[18] << 8) + (packet->payload[19])) * 0.001);
  whill->left_motor.angle = (float)((signed short)((packet->payload[20] << 8) + (packet->payload[21])) * 0.001);

  whill->right_motor.speed = (signed short)((packet->payload[22] << 8)+(packet->payload[23])) * 4;
  whill->left_motor.speed  = (signed short)((packet->payload[24] << 8)+(packet->payload[25])) * 4;

  whill->power                = packet->payload[26] == 0x00 ? false : true;
  whill->speed_mode_indicator = packet->payload[27];

  // Experimental, Set interval comming from Motor Controller
  if (packet->rawLength() == WHILL::Packet::DATASET1_LEN_V02){
    uint8_t time_ms = packet->payload[29];
    if (whill->past_time_ms < 0)whill->_interval = 0;
    else whill->_interval = WHILL::calc_time_diff(whill->past_time_ms,time_ms);
    whill->past_time_ms = time_ms;
  }
  else
  {
    whill->past_time_ms = 0;
    whill->_interval = -1;
  }

  if(whill == NULL)return;
  whill->fire_callback(CALLBACK_DATA1);
}

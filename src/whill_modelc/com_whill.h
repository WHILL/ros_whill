/*
MIT License

Copyright (c) 2018 WHILL inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * Functions to communcate with WHILL via UART
 * author : Kazumichi Shirai
 */
#include <string.h>

int initializeComWHILL(int *fd,std::string port);
void closeComWHILL(int fd);
int sendJoystickStop(int fd);
//int sendSpeedDown(int fd, int rate); // removed on Model C
int recvDataWHILL(int fd, char recv_buf[]);
int sendJoystick(int fd, char fb, char lr);
int releaseJoystick(int fd);
//int sendStartSendingData(int fd, int t, char data_set_num); 
int sendStartSendingData(int fd, int t, char data_set_num, char speed_mode);  // modified on Model C
int sendStopSendingData(int fd);
//int sendSeatMove(int fd, char direction); // removed on Model C
int sendPowerOn(int fd);
int sendPowerOff(int fd);
//int sendSetForward(int fd, char max_speed, char accel, char deceel); // removed on Model C
//int sendSetReverse(int fd, char max_speed, char accel, char deccel); // removed on Model C
//int sendSetTurn(int fd, char max_speed, char accel, char deccel); // removed on Model C
int sendSetSpeed(int fd, char s1, char fm1, char fa1, char fd1, char rm1, char ra1, char rd1, char tm1, char ta1, char td1); // added on Model C
int sendSetBatteryOut(int fd, char battery_out); // added on Model C


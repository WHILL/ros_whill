/*
 * Functions to communcate with WHILL via UART
 * author : Kazumichi Shirai
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include "./uart.h"

#define NUM_ADD_COM (3) //Add "protocol sign". "length of data" and "checksum"
//Comand ID List
enum{
     START = 0,
     STOP,
     SET_POWER,
     //SET_SEAT_MOVE, 		// removed on Model C
     SET_JOYSTICK,
     //SET_FORWARD, 		// removed on Model C
     //SET_REVERSE, 		// removed on Model C
     //SET_TURN, 		// removed on Model C
     //SET_SPEED_DOWN, 		// removed on Model C
     SET_SPEED_PROFILE, 	// added on Model C
     SET_BATTERY_VOLTAGE_OUT, 	// added on Model C
};

#define USER_CTRL_DISABLE (0)
#define USER_CTRL_ENABLE (1)
#define ENABLE (1)

int initializeComWHILL(int *fd)
{
     if(initializeUART(fd) < 0){
	  fprintf(stderr, "Can't initizalize UART to communicate with WHILL\n");
	  return -1;
     }
     
     return 1;
}

void closeComWHILL(int fd)
{
     closeUART(fd);
     close(fd);
}

int sendWHILLCmd(int fd, char cmd_data[], int length)
{
     int i;
     const int num_data = length + NUM_ADD_COM;
     char data[num_data];
     int checksum = 0;

     data[0] = PROTOCOL_SIGN;
     data[1] = length + 1; //length of cmd_data + 1(=checksum)
     for(i=0;i<length;i++){
       data[i+2] = cmd_data[i];
     }
     for(i=0;i<(num_data-1);i++){
	  checksum ^= data[i];
     }
     data[num_data-1] = checksum;
     if(sendCmdUART(fd, data, num_data) < 0){
	  fprintf(stderr, "Failed to send command No %d\n", data[2]);
	  return -1;
     }
     return 1;

}

int sendJoystickStop(int fd)
{
     const int num_cmd = 4;
     char cmd[num_cmd];

     cmd[0] = SET_JOYSTICK;
     cmd[1] = USER_CTRL_DISABLE;
     cmd[2] = 0x00;
     cmd[3] = 0x00;
     return sendWHILLCmd(fd, cmd, num_cmd);
}

/* removed on Model C
int sendSpeedDown(int fd, int rate)
{
     const int num_cmd = 2;
     char cmd[num_cmd];

     cmd[0] = SET_SPEED_DOWN;
     cmd[1] = rate; //joystic input is divided by 8

     return sendWHILLCmd(fd, cmd, num_cmd);
}
*/

int recvDataWHILL(int fd, char recv_buf[])
{
     return recvDataUART(fd, recv_buf);
}

int sendJoystick(int fd, char fb, char lr)
{
     const int num_cmd = 4;
     char cmd[num_cmd];

     cmd[0] = SET_JOYSTICK;
     cmd[1] = USER_CTRL_DISABLE;
     cmd[2] = fb;
     cmd[3] = lr;
     return sendWHILLCmd(fd, cmd, num_cmd);
}	  

int releaseJoystick(int fd)
{
     const int num_cmd = 4;
     char cmd[num_cmd];

     cmd[0] = SET_JOYSTICK;
     cmd[1] = USER_CTRL_ENABLE;
     cmd[2] = 0;
     cmd[3] = 0;
     return sendWHILLCmd(fd, cmd, num_cmd);
}	  

//int sendStartSendingData(int fd, int t, char data_set_num)
int sendStartSendingData(int fd, int t, char data_set_num, char speed_mode) // modified on Model C
{
     //const int num_cmd = 4;
     const int num_cmd = 5; // modified on Model C
     char cmd[num_cmd];

     cmd[0] = START;
     cmd[1] = data_set_num;
     cmd[2] = (char)(t >> 8);
     cmd[3] = (char)t;
     cmd[4] = speed_mode; // added on Model C

     return sendWHILLCmd(fd, cmd, num_cmd);
}

int sendStopSendingData(int fd)
{
     const int num_cmd = 1;
     char cmd[num_cmd];

     cmd[0] = STOP;
     return sendWHILLCmd(fd, cmd, num_cmd);
}

/* removed on Model C
int sendSeatMove(int fd, char direction)
{
     const int num_cmd = 3;
     char cmd[num_cmd];

     cmd[0] = SET_SEAT_MOVE;
     cmd[1] = USER_CTRL_DISABLE;
     cmd[2] = direction;

     return sendWHILLCmd(fd, cmd, num_cmd);
}
*/

int sendPowerOn(int fd)
{
     const int num_cmd = 2;
     char cmd[num_cmd];

     cmd[0] = SET_POWER;
     cmd[1] = 1;

     return sendWHILLCmd(fd, cmd, num_cmd);
}     

int sendPowerOff(int fd)
{
     const int num_cmd = 2;
     char cmd[num_cmd];

     cmd[0] = SET_POWER;
     cmd[1] = 0;

     return sendWHILLCmd(fd, cmd, num_cmd);
}     

/* removed on Model C
int sendSetForward(int fd, char max_speed, char accel, char deccel)
{
     const int num_cmd = 4;
     char cmd[num_cmd];

     cmd[0] = SET_FORWARD;
     cmd[1] = max_speed;
     cmd[2] = accel;
     cmd[3] = deccel;

     return sendWHILLCmd(fd, cmd, num_cmd);
}     

int sendSetReverse(int fd, char max_speed, char accel, char deccel)
{
     const int num_cmd = 4;
     char cmd[num_cmd];

     cmd[0] = SET_REVERSE;
     cmd[1] = max_speed;
     cmd[2] = accel;
     cmd[3] = deccel;

     return sendWHILLCmd(fd, cmd, num_cmd);
}     

int sendSetTurn(int fd, char max_speed, char accel, char deccel)
{
     const int num_cmd = 4;
     char cmd[num_cmd];

     cmd[0] = SET_TURN;
     cmd[1] = max_speed;
     cmd[2] = accel;
     cmd[3] = deccel;

     return sendWHILLCmd(fd, cmd, num_cmd);
}
*/

// added on Model C
int sendSetSpeed(int fd, char s1, char fm1, char fa1, char fd1, char rm1, char ra1, char rd1, char tm1, char ta1, char td1)
{
     const int num_cmd = 11;
     char cmd[num_cmd];

     cmd[0] = SET_SPEED_PROFILE;
     cmd[1] = s1;
     cmd[2] = fm1;
     cmd[3] = fa1;
     cmd[4] = fd1;
     cmd[5] = rm1;
     cmd[6] = ra1;
     cmd[7] = rd1;
     cmd[8] = tm1;
     cmd[9] = ta1;
     cmd[10] = td1;

     return sendWHILLCmd(fd, cmd, num_cmd);
}

// added on Model C
int sendSetBatteryOut(int fd, char battery_out)
{
     const int num_cmd = 2;
     char cmd[num_cmd];

     cmd[0] = SET_BATTERY_VOLTAGE_OUT;
     cmd[1] = battery_out;

     return sendWHILLCmd(fd, cmd, num_cmd);
}


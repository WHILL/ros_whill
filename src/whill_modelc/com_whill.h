/*
 * Functions to communcate with WHILL via UART
 * author : Kazumichi Shirai
 */
int initializeComWHILL(int *fd);
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


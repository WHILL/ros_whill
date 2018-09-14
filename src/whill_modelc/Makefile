#Makefile.c
#all: ctrl_whill power_on power_off get_data set_joystick stop_joystick set_speed_params move_seat # modified on Model C
all: ctrl_whill power_on power_off get_data set_joystick stop_joystick set_speed_profile set_batteryout
.c.o:
	gcc -c $*.c

ctrl_whill : main.o com_whill.o uart.o
	gcc -o ctrl_whill main.o com_whill.o uart.o

power_on : power_on.o com_whill.o uart.o
	gcc -o power_on power_on.o com_whill.o uart.o

power_off : power_off.o com_whill.o uart.o
	gcc -o power_off power_off.o com_whill.o uart.o

get_data : get_data.o com_whill.o uart.o
	gcc -o get_data get_data.o com_whill.o uart.o

set_joystick : set_joystick.o com_whill.o uart.o
	gcc -o set_joystick set_joystick.o com_whill.o uart.o

stop_joystick : stop_joystick.o com_whill.o uart.o
	gcc -o stop_joystick.exe stop_joystick.o com_whill.o uart.o

set_speed_profile : set_speed_profile.o com_whill.o uart.o
	gcc -o set_speed_profile set_speed_profile.o com_whill.o uart.o

#move_seat : move_seat.o com_whill.o uart.o # removed on Model C
#	gcc -o move_seat move_seat.o com_whill.o uart.o # removed on Model C

set_batteryout : set_batteryout.o com_whill.o uart.o
	gcc -o set_batteryout set_batteryout.o com_whill.o uart.o

main.o : main.c com_whill.h
com_whill.o : com_whill.c com_whill.h uart.h
uart.o : uart.c uart.h
clean:
	rm -f *.o *~

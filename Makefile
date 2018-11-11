# MPLAB IDE generated this makefile for use with GNU make.
# Project: SelfBalancer.mcp
# Date: Tue Dec 02 16:45:17 2014

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

SelfBalancer.hex : SelfBalancer.cof
	$(HX) "SelfBalancer.cof"

SelfBalancer.cof : inv_pendu.o
	$(CC) -mcpu=33FJ128MC802 "inv_pendu.o" -o"SelfBalancer.cof" -Wl,--script="..\..\..\..\Program Files (x86)\Microchip\MPLAB C30\support\dsPIC33F\gld\p33FJ128MC802.gld",--defsym=__MPLAB_BUILD=1,-Map="SelfBalancer.map",--report-mem

inv_pendu.o : ../../../../program\ files\ (x86)/microchip/mplab\ c30/include/string.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/include/ctype.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/include/stdlib.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/include/math.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/include/stdarg.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/include/stddef.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/include/stdio.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/libpic30.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h inv_pendu.c
	$(CC) -mcpu=33FJ128MC802 -x c -c "inv_pendu.c" -o"inv_pendu.o" -g -Wall

clean : 
	$(RM) "inv_pendu.o" "SelfBalancer.cof" "SelfBalancer.hex"


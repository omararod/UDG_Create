#ifndef USBLAYER_H
#define USBLAYER_H
#define GET_SENSORS_REPORT_CODE 0x37

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>

#include <unistd.h>
#include <stdarg.h>
#include "UDG_Create.h"

int initializeUSB(int VID, int PID);
void printUSBMessage(const char* message,...);
void GetUSBData(unsigned char* destinationBuffer);
void CloseUSB();
void printUSBMessage(const char* message,...);
void SetUSBVerbosity(t_verbosity level);
#endif

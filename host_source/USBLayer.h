#ifndef USBLAYER_H
#define USBLAYER_H
#define GET_SENSORS_REPORT_CODE 0x37

#include "UDG_Create.h"
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef __linux
#include <unistd.h>
#else

#include <Windows.h>
#include <Dbt.h>
#include <SetupAPI.h>
#include <string.h>


//link with setupapi.lib
#pragma comment(lib,"setupapi.lib")
//ID for our device
#define VID_PID L"HID\\VID_04D8&PID_003F"//&REV_0002


#endif


int initializeUSB(int VID, int PID);
void printUSBMessage(const char* message,...);
void GetUSBData(unsigned char* destinationBuffer);
void CloseUSB();
void printUSBMessage(const char* message,...);
void SetUSBVerbosity(t_verbosity level);
#endif

#ifndef SERIALPORT_H
#define SERIALPORT_H

//#define ERROR -1

#ifndef __linux
#include <Windows.h>
#else
#include <termios.h>	//UNIX API for terminal I/O
#include <unistd.h> 	//close() function
#endif

#include <stdio.h>

#include <fcntl.h>	// Constant declarations for termios functions

#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <string.h>	//needed for memset
#include <thread>
#include <stdarg.h>
#include "UDG_Create.h"
using namespace std;

#ifndef __linux
HANDLE AutoOpenPort();
HANDLE OpenPort(LPWSTR portName);
int WriteToSerial(HANDLE portDescriptor, unsigned char* buffer, int numberOfBytes);
int ReadFromSerial(HANDLE portDescriptor, unsigned char* buffer, int numberOfBytes);
#else
int AutoOpenPort();
int OpenPort(const char* portName);
int ConfigurePort(int portDescriptor);
int WriteToSerial(int portDescriptor, unsigned char* buffer, int numberOfBytes);
int ReadFromSerial(int portDescriptor, unsigned char* buffer, int numberOfBytes);
#endif

void PrintError(int errorCode);

#ifndef __linux
void StartThreadedRead(HANDLE portDescriptor, unsigned char* buffer, int numberOfBytes);
void ThreadedRead(HANDLE portDescriptor, unsigned char* buffer, int numberOfBytes);
#else
void StartThreadedRead(int portDescriptor, unsigned char* buffer, int numberOfBytes);
void ThreadedRead(int portDescriptor, unsigned char* buffer, int numberOfBytes);
#endif

void printSerialMessage(const char* message,...);
void SetSerialPortVerbosity(t_verbosity level);

enum Errors
{
	ERROR_OPEN,
	ERROR_SET_SPEED,
	ERROR_CONFIGURE_PORT,
	ERROR_WRITE,
	ERROR_READ

};


#endif

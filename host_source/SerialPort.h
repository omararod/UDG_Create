#ifndef SERIALPORT_H
#define SERIALPORT_H
#define ERROR -1

#include <termios.h>	//UNIX API for terminal I/O
#include <fcntl.h>	// Constant declarations for termios functions
#include <unistd.h> 	//close() function
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <string.h>	//needed for memset
#include <thread>
#include <stdarg.h>
#include "UDG_Create.h"
using namespace std;

void PrintError(int errorCode);
int OpenPort(const char* portName);
int ConfigurePort(int portDescriptor);
int WriteToSerial(int portDescriptor,unsigned char* buffer,int numberOfBytes);
int ReadFromSerial(int portDescriptor,unsigned char* buffer,int numberOfBytes);
void StartThreadedRead(int portDescriptor,unsigned char* buffer,int numberOfBytes);
void ThreadedRead(int portDescriptor,unsigned char* buffer,int numberOfBytes);
void printSerialMessage(const char* message,...);
void SetSerialPortVerbosity(t_verbosity level);
int AutoOpenPort();
enum Errors
{
	ERROR_OPEN,
	ERROR_SET_SPEED,
	ERROR_CONFIGURE_PORT,
	ERROR_WRITE,
	ERROR_READ

};


#endif

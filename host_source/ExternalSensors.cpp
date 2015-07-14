#include "USBLayer.h"
#define VID 0X04D8
#define PID 0X003F

bool InitializeSensors()
{
	if(initializeUSB(VID,PID) < 0)
	{return false;}
	else
	{return true;}
}

bool InitializeSensors(int vid,int pid)
{
	if(initializeUSB(vid,pid) < 0)
	{return false;}
	else
	{return true;}
}

void GetSensors(int destinationArray[NUMBER_OF_SENSORS])
{
	unsigned char buffer[65];
#ifndef __linux

#endif
	GetUSBData(buffer);
	int j=0;
#ifndef __linux
	for(int i=2 ; i<28;i+=2)
#else
	for (int i = 1; i<27; i += 2)
#endif
	{
		destinationArray[j]=(buffer[i+1]<<8)+buffer[i];
		j++;
	}
	destinationArray[13] =  
#ifndef __linux
		(int)buffer[28];
#else
		(int)buffer[27];
#endif
}

int GetNthSensor(int n)
{
	if(n < 0 || n > 13)
	{return -1;}	
	int array[NUMBER_OF_SENSORS];
	GetSensors(array);
	return array[n];
}

void CleanExternalSensors()
{
	CloseUSB();
}

void SetSensorVerbosity(t_verbosity level)
{
	SetUSBVerbosity(level);
}

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
	GetUSBData(buffer);
	int j=0;
	for(int i=1 ; i<27;i+=2)
	{
		destinationArray[j]=(buffer[i+1]<<8)+buffer[i];
		j++;
	}
	destinationArray[13] =  (int)buffer[27]; 
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

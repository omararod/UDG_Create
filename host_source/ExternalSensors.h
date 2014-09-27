#ifndef EXTERNALSENSORS_H
#define EXTERNALSENSORS_H
//#include "DataTypes.h"
#include "UDG_Create.h"
bool InitializeSensors();
bool InitializeSensors(int VID, int PID);
void GetSensors(int destinationArray[13]);
int GetNthSensor(int n);
void CleanExternalSensors();
void SetSensorVerbosity(t_verbosity level);

#endif

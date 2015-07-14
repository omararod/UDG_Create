#include "UDG_Create.h"
#include<thread>
#include "ExternalSensors.h"
#include "SerialPort.h"
#ifndef __linux
#include <Windows.h>
#undef ConfigurePort
#endif

using namespace std;
//sleep for x milliseconds, this function is suposed to be cross-platform
void SleepMS(int x)
{
#ifdef __linux
	usleep(x * 100);
#else
	Sleep(x);
#endif
}


//************NOT PART OF CREATE CLASS***************************************
//This function will save the numberOfBytes bytes in buffer when requested
//It's the user's responsability to give the proper interpretation to the raw data
#ifdef __linux
void ThreadedReadStream(int portDescriptor,Create* r,unsigned char*buffer,int numberOfBytes)
#else
void ThreadedReadStream(HANDLE portDescriptor, Create* r, unsigned char*buffer, int numberOfBytes)
#endif
{
	while(r->getStreamingState())
	{
		//usleep(15000);
		SleepMS(15);
		ReadFromSerial(portDescriptor,buffer,numberOfBytes);
	}
}
//***********************************************************
int16 Create::toInt16(int integer)
{
	int16 data;
	unsigned char temp[2];
	memcpy(temp,&integer,2);
	data.H = temp[1];
	data.L = temp[0];
	return data;
	
}


Create::Create()
{
	
	commonInitializationProcedures(string("nothing"),true);
}

Create::Create(string _portName)
{
	setVerbosity(VERBOSITY_NORMAL);
	portName = _portName;
	commonInitializationProcedures(_portName, false);
}
Create::Create(string _portName, t_verbosity level)
{
	setVerbosity(level);	
	portName = _portName;
	commonInitializationProcedures(_portName,false);

}

void Create::commonInitializationProcedures(string _portName,bool automaticInitialization)
{
		
	if(automaticInitialization)
	{
		portDescriptor = AutoOpenPort();
		
	}
	else
	{
		portName = _portName;
#ifndef __linux
		wstring aux;
		aux.assign(portName.begin(), portName.end());
		portDescriptor = OpenPort((LPWSTR)aux.c_str());
#else
		portDescriptor = OpenPort(portName.c_str());
#endif
	}
#ifdef __linux
	ConfigurePort(portDescriptor);
#endif
	streamingState = false;
	externalSensorsEnabled = InitializeSensors(); //<--- USB
	if(!externalSensorsEnabled)
	{cout<< "USB external sensors not enabled!"<<endl;}
	start();
}
Create::~Create()
{
	if(externalSensorsEnabled)
	{CleanExternalSensors();}
}

string Create::getPortName()
{
	return portName;
}

bool Create::isCharging()
{
	//TODO: implement charging state by hardware
	return charging;
}

void Create::start()
{
	unsigned char ins = 0x80;
	WriteToSerial(portDescriptor,&ins,1);
	mode = PASSIVE;
}

void Create::baud(unsigned char baudCode)
{
	if(baudCode>=0 && baudCode<=11)
	{
		unsigned char ins[2]{0x81,baudCode};
		WriteToSerial(portDescriptor,ins,2);
		//according to documentation you must wait
		//100ms before sending another instruction
		//usleep(100000);
		SleepMS(100);
		baudRate = getBaudCode(baudCode);
	}
	else
		error(INVALID_BAUDRATE,&baudCode);
}

void Create::control()
{
	safe();
}

void Create::safe()
{
	unsigned char ins = 0x83;
	WriteToSerial(portDescriptor,&ins,1);
	mode = SAFE;
}

void Create::full()
{
	unsigned char ins = 0x84;
	WriteToSerial(portDescriptor,&ins,1);
	mode = FULL;
}

void Create::spot()
{
	unsigned char ins = 0x86;
	WriteToSerial(portDescriptor,&ins,1);
	mode = PASSIVE;
}

void Create::cover()
{
	unsigned char ins = 0x87;
	WriteToSerial(portDescriptor,&ins,1);
	mode = PASSIVE;
}

void Create::demo(unsigned char demo)
{
	if(demo>0 && demo<=9)
	{
		unsigned char ins[2] = {0x88,demo};
		WriteToSerial(portDescriptor,ins,2);
		mode = PASSIVE;
	}
	else
		error(INVALID_DEMO,&demo);
}

void Create::drive(int velocity,int radius)
{
	if(mode == SAFE || mode == FULL)
	{
		int16 data1 = toInt16(velocity);
		int16 data2 = toInt16(radius);
		unsigned char ins[5] = {0x89,data1.H,data1.L,data2.H,data2.L};
		WriteToSerial(portDescriptor,ins,5);		
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function DRIVE");
}

void Create::lowSideDrivers(unsigned char driverBits)
{
	//Probar funcion 1/7
	if(mode == SAFE || mode == FULL)
	{
		if(driverBits>=0 && driverBits<=7)
		{
			unsigned char ins[2] = {0x8A,driverBits};
			WriteToSerial(portDescriptor,ins,2);
		}
		else
			error(OUT_OF_RANGE,(char*)"Function LOWSIDEDRIVERS");
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function LOWSIDEDRIVERS");
}

void Create::leds(unsigned char bit,unsigned char color,unsigned char intensity)
{
	if(mode == SAFE || mode == FULL)
	{
		if(bit == 2 || bit == 8 || bit == 10 || bit == 0)
		{
			unsigned char ins[4]{0x8B,bit,color,intensity};
			WriteToSerial(portDescriptor,ins,4);
		}
		else
		{
			error(OUT_OF_RANGE,(char*)"Function LEDS");
		}
	}
	else
	{ 
		string auxString(charMode(mode));	
		auxString += " in Function LEDS"; 
		error(INVALID_INSTRUCTION_MODE,const_cast<char*>(auxString.c_str()));	
	}
}



void Create::song(unsigned char songNumber,unsigned char n,...)
{
	unsigned char* ins = new unsigned char[n*2+3];
	ins[0] = (unsigned char)0x8C;
	ins[1] = songNumber;
	ins[2] = n;
	
	va_list args;
	va_start(args,n);
	if(songNumber>=0 && songNumber<=15 && n>=1 && n<=16)
	{
		for(int i=3;i<n*2+3;i+=2)
		{
			ins[i] = (unsigned char)va_arg(args,int);
			ins[i+1] = (unsigned char)va_arg(args,int);
		}
		WriteToSerial(portDescriptor,ins,n*2+3);
	}
	else
		error(OUT_OF_RANGE,(char*)"Function song(unsigned char, unsigned char,...)");
}

void Create::playSong(unsigned char songNumber)
{
	if(mode == SAFE || mode == FULL)
	{
		if(songNumber>=0 && songNumber<=15)
		{
			unsigned char ins[2]{0x8D,songNumber};
			WriteToSerial(portDescriptor,ins,2);
		}
		else
		{
			error(OUT_OF_RANGE,(char*)"Function PLAYSONG");
		}
	}
	else
	{ 
		string auxString(charMode(mode));	
		auxString += " in Function PLAYSONG"; 
		error(INVALID_INSTRUCTION_MODE,const_cast<char*>(auxString.c_str()));	
	}
}

int Create::sensors(unsigned char idPacket)
{
	if(idPacket>=0 && idPacket<=42)
	{
		unsigned char ins[2]{0x8E,idPacket};
		WriteToSerial(portDescriptor,ins,2);
		int sizePacket = getSizePacket(idPacket);
		return updateSensor(idPacket,sizePacket);
	}
	else
		error(OUT_OF_RANGE,(char*)"Function SENSORS");
}

int Create::getSizePacket(int idPacket)
{
	int sizePacket = 0;
	switch(idPacket)
	{
		case 0:
		sizePacket = 26;
		break;

		case 1:
		sizePacket = 10;
		break;		

		case 2:
		sizePacket = 6;
		break;	

		case 3:
		sizePacket = 10;
		break;

		case 4:
		sizePacket = 14;
		break;

		case 5:
		sizePacket = 12;
		break;

		case 6:
		sizePacket = 52;
		break;
		
		case 7:case 8:case 9:case 10:case 11:case 12:case 13:
		case 14:case 15:case 16:case 17:case 18:case 21:case 24:
		case 32:case 34:case 35:case 36:case 37:case 38:
			sizePacket = 1;
		break;
		case 19:case 20:case 22:case 23:case 25:case 26:case 27:case 28:
		case 29:case 30:case 31:case 33:case 39:case 40:case 41:case 42:
			sizePacket = 2;
		break;
	}
	return sizePacket;
}

void Create::coverAndDock()
{
	unsigned char ins = 0x8F;
	WriteToSerial(portDescriptor,&ins,1);
	mode = PASSIVE;
}

void Create::pwmLowSideDrivers(unsigned char lowSideDriver2,unsigned char lowSideDriver1,unsigned char lowSideDriver0)
{
	//Probar funcion 2/7
	if(mode == SAFE || mode == FULL)
	{
		if(lowSideDriver2<=128 && lowSideDriver2>=0 &&
		lowSideDriver1<=128 && lowSideDriver1>=0 &&
		lowSideDriver0<=128 && lowSideDriver0>=0)
		{
			unsigned char ins[4] = {0x90,lowSideDriver2,lowSideDriver1,lowSideDriver0};
			WriteToSerial(portDescriptor,ins,4);
		}
		else
			error(OUT_OF_RANGE,(char*)"Function PWMLOWSIDEDRIVERS");
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function PWMLOWSIDEDRIVERS");
}

void Create::driveDirect(int rightVelocity,int leftVelocity)
{
	if(mode == SAFE || mode == FULL)
	{
		int16 data1 = toInt16(rightVelocity);
		int16 data2 = toInt16(leftVelocity);
		unsigned char ins[5] = {0x91,data1.H,data1.L,data2.H,data2.L};
		WriteToSerial(portDescriptor,ins,5);		
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function DRIVEDIRECT");
}

void Create::digitalOutputs(unsigned char outputBits)
{
	//Probar funcion 3/7
	if(mode == SAFE || mode == FULL)
	{
		if(outputBits>=0 && outputBits<=7)
		{
			unsigned char ins[2] = {0x93,outputBits};
			WriteToSerial(portDescriptor,ins,2);
		}
		else
			error(OUT_OF_RANGE,(char*)"Function DIGITALOUTPUTS");
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function DIGITALOUTPUTS");
}

//TODO: I probably need to re-implement this
/*void Create::stream(vector<unsigned char> packets)
{
	//Probar funcion 4/7
	if(mode==PASSIVE || mode == SAFE || mode == FULL)
	{
		unsigned char* ins = new unsigned char[packets.size()+2];
		ins[0] = 0x94;
		ins[1] = packets.size();
		int size = packets.size();
		for(int i = 2;i<size;i++)
		{
			ins[i] = packets[i-2];
		}
		WriteToSerial(portDescriptor,ins,size+2);
		streamingState = true;
		//Read packets
		 sleep(1);
		 std::thread t(ThreadedReadStream,portDescriptor,ins,8);
				t.join();
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Funcion STREAM");
}*/

void Create::stream(unsigned char* destinationBuffer,void* thread,int n,...)
{
	std::thread *t =(std::thread*)t;
	va_list args;
	va_start(args,n);
	int bytesToRead=3+n;
	//Probar funcion 4/7
	if(n>=0 && n<=43)
	{
	if(mode==PASSIVE || mode == SAFE || mode == FULL)
	{
		unsigned char* ins = new unsigned char[n+2];
		ins[0] = 0x94;
		ins[1] =n;
		
		for(int i = 2;i<n+2;i++)
		{
			ins[i] = (unsigned char)va_arg(args,int);
			bytesToRead+=getSizePacket(ins[i]);
		}
		WriteToSerial(portDescriptor,ins,n+2);
		streamingState = true;
		//Read packets
		 //sleep(1);
		SleepMS(1000);
		  *t= std::thread(ThreadedReadStream,portDescriptor,this,destinationBuffer,bytesToRead);
				//t.join();
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function STREAM");
	}
	else
	{
	error(OUT_OF_RANGE,(char*)"Function DIGITALOUTPUTS");
	}
}

void Create::queryList(vector<unsigned char> packets)
{
	//Probar funcion 5/7
	if(mode==PASSIVE || mode == SAFE || mode == FULL)
	{
		if(packets.size()>=1 && packets.size()<=255)
		{
			unsigned char* ins = new unsigned char[packets.size()+2];
			ins[0] = 0x95;
			ins[1] = packets.size();
			int size = packets.size();
			for(int i = 2;i<size;i++)
			{
				ins[i] = packets[i-2];
			}
			WriteToSerial(portDescriptor,ins,size+2);
			//Read packet
			
		}
		else
			error(OUT_OF_RANGE,(char*)"Function QUERYLIST");
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function QUERYLIST");
}

void Create::pauseResumeStream(bool streamState)
{
	
	if(mode == PASSIVE || mode == SAFE || mode == FULL)
	{
		unsigned char ins[2] = {0x96,streamState};
		WriteToSerial(portDescriptor,ins,2);
		streamingState = streamState;
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function PAUSERESUMESTREAM");
}

void Create::sendIr(unsigned char byteValue)
{
	//Probar funcion 6/6
	if(mode == SAFE || mode == FULL)
	{
		unsigned char ins[2] = {0x97,byteValue};
		WriteToSerial(portDescriptor,ins,2);
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function SENDIR");
}

void Create::script(vector<unsigned char> instructions)
{
	if(mode == PASSIVE || mode == SAFE || mode == FULL)
	{
		if(instructions.size()>=1 && instructions.size()<=100)
		{
			unsigned char* ins = new unsigned char[instructions.size()+2];
			ins[0] = 0x98;
			ins[1] = instructions.size();
			int size = instructions.size();
			for(int i = 2;i<size;i++)
			{
				ins[i] = instructions[i-2];
			}
			WriteToSerial(portDescriptor,ins,size+2);
		}
		else
		{error(OUT_OF_RANGE,(char*)"Function SCRIPT");}
	}
	else
	{error(INVALID_INSTRUCTION_MODE,(char*)"Function SCRIPT");}
		
}

void Create::script(unsigned char n,...)
{
	va_list args;
	va_start(args,n);
	if(mode == PASSIVE || mode == SAFE || mode == FULL)
	{
		if(n >= 1 && n <= 100)
		{
			unsigned char* ins = new unsigned char[n + 2];
			ins[0] = 0x98;
			ins[1] = n;
			for(int i = 2;i<n;i++)
			{
				ins[i] = (unsigned char)va_arg(args,int);
			}
			WriteToSerial(portDescriptor,ins,n + 2);
		}
		else
		{error(OUT_OF_RANGE,(char*)"Function SCRIPT");}
	}
	else
	{error(INVALID_INSTRUCTION_MODE,(char*)"Function SCRIPT");}
		
}
void Create::playScript()
{
	if(mode == PASSIVE || mode == SAFE || mode == FULL)
	{	
		unsigned char ins = 0x99;
		WriteToSerial(portDescriptor,&ins,1);
	}
	else
	{error(INVALID_INSTRUCTION_MODE,(char*)"Function SCRIPT");}
}

void Create::showScript()
{
	if(mode == PASSIVE || mode == SAFE || mode == FULL)
	{
		unsigned char ins = 0x9A;
		WriteToSerial(portDescriptor,&ins,1);
		unsigned char ins2[101];
		//usleep(120000);
		SleepMS(120);
		ReadFromSerial(portDescriptor,ins2,101);
		int size = (int)ins2[0];
		for(int i=0;i<size;i++)
		{
			printRobotMessage("%i ",(int)ins2[i]);
		}
		printRobotMessage("\n");

		if(streamingState)
		{streamingState = false;}
	}
	else
	{error(INVALID_INSTRUCTION_MODE,(char*)"Function SCRIPT");}
}

void Create::waitTime(unsigned char time)
{
	if(mode == PASSIVE || mode == SAFE || mode == FULL)
	{
	unsigned char ins[2]{0x9B,time};
	WriteToSerial(portDescriptor,ins,2);
	}
	else
	{error(INVALID_INSTRUCTION_MODE,(char*)"Function SCRIPT");}
}

void Create::waitDistance(int distance)
{
	if(mode == FULL || mode == SAFE || mode == PASSIVE)
	{
		int16 data1 = toInt16(distance);
		unsigned char ins[3]{0x9C,data1.H,data1.L};
		WriteToSerial(portDescriptor,ins,3);
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function WAITDISTANCE");
}

void Create::waitAngle(int angle)
{
	if(mode == FULL || mode == SAFE || mode == PASSIVE)
	{
		int16 data1 = toInt16(angle);
		unsigned char ins[3]{0x9D,data1.H,data1.L};
		WriteToSerial(portDescriptor,ins,3);
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function WAITANGLE");
}

void Create::waitEvent(unsigned char event)
{
	if(mode == FULL || mode == SAFE || mode == PASSIVE)
	{
		if(event >= -20 && event <=20)
		{
			unsigned char ins[2]{0x9E,event};
			WriteToSerial(portDescriptor,ins,3);
		}
		else
		{error(OUT_OF_RANGE,(char*)"Function WAITEVENT");}
	}
	else
		error(INVALID_INSTRUCTION_MODE,(char*)"Function WAITEVENT");
}

void Create::error(int nError,void *extra)
{
	int* extraAp = (int*)extra;
	char* extraInfoString = (char*)extra;
	if(robotVerbosity == VERBOSITY_NORMAL)
	{	switch(nError)
		{
		case INVALID_DEMO:
			cout<<"Invalid Demo(1-9):\t"<<*extraAp<<endl;
		break;
		case INVALID_BAUDRATE:
			cout<<"Invalid Baud code (0-11)\t"<<*extraAp<<endl;
		break;
		case INVALID_INSTRUCTION_MODE:
			cout<<"The instruction cannot be executed in this operation mode:\t"<<extraInfoString<<endl;
		break;
		case OUT_OF_RANGE:
			cout<<"Parameter is out of range in "<<extraInfoString<<endl;
		break;
		case INVALID_MODE:
			cout<<"Selected mode doesn't exist in charMode function"<<endl;
		break;
		//case EXTRENAL_SENSORS_ERROR:
		//	cout<<"Hubo un problema al inicializar los sensores externos. Contiunando con los sensores desactivados"<<endl;
		//break;
		default:
			cout<<"Unknown error"<<endl;
		}
	}
}

int Create::getBaudCode(int baudCode)
{
	//functions returns 0 if invalid
	int tmp = 0;
	switch(baudCode)
	{
	case BAUD300:
		tmp = 300;
	break;
	case BAUD600:
		tmp = 600;
	break;
	case BAUD1200:
		tmp = 1200;
	break;
	case BAUD2400:
		tmp = 2400;
	break;
	case BAUD4800:
		tmp = 4800;
	break;
	case BAUD9600:
		tmp = 9600;
	break;
	case BAUD14400:
		tmp = 14400;
	break;
	case BAUD19200:
		tmp = 19200;
	break;
	case BAUD28800:
		tmp = 28800;
	break;
	case BAUD38400:
		tmp = 38400;
	break;
	case BAUD57600:
		tmp = 57600;
	break;
	case BAUD115200:
		tmp = 115200;
	break;
	default:
		error(INVALID_BAUDRATE,&baudCode);
	}
	return tmp;
}

char* Create::charMode(int mode)
{
	//La funcion regresa elcuando es incorrecto el modo
	char * tmp = (char*)"OFF";
	switch(mode)
	{
		case OFF:
			tmp = (char*)"OFF";
		break;
		case FULL:
			tmp = (char*)"FULL";
		break;
		case PASSIVE:
			tmp = (char*)"PASSIVE";
		break;
		case SAFE:
			tmp = (char*)"SAFE";
		break;
		default:
			error(INVALID_MODE,(char*)" ");
	}
	return tmp;
}

//-------------------------------------
//Metodos get
//--------------------------------------

bool Create::getBumpRight()
{
	return bumpRight;
}

bool Create::getBumpLeft()
{
	return bumpLeft;
}

bool Create::getWheelDropRight()
{
	return wheelDropRight;
}

bool Create::getWheelDropLeft()
{
	return wheelDropLeft;
}

bool Create::getWheelDropCaster()
{
	return wheelDropCaster;
}

bool Create::getWallSeen()
{
	return wall;
}

bool Create::getCliffLeft()
{
	return cliffLeft;
}

bool Create::getCliffFrontLeft()
{
	return cliffFrontLeft;
}

bool Create::getCliffFrontRight()
{
	return cliffFrontRight;
}

bool Create::getCliffRight()
{
	return cliffRight;
}

bool Create::getVirtualWall()
{
	return virtualWall;
}

bool Create::getLd0()
{
	return ld0;
}

bool Create::getLd1()
{
	return ld1;
}

bool Create::getLd2()
{
	return ld2;
}

bool Create::getRightWheel()
{
	return rightWheel;
}

bool Create::getLeftWheel()
{
	return leftWheel;
}

//Unused bytes 15-16.

unsigned char Create::getInfraredByte()
{
	return infraredbyte;
}

bool Create::getAdvanceBtn()
{
	return advancebtn;
}

bool Create::getPlayBtn()
{
	return playbtn;
}

int Create::getDistance()
{
	return distance;
}

int Create::getAngle()
{
	return angle;
}

unsigned char Create::getChargingState()
{
	return chargingstate;
}

int Create::getVoltage()
{
	return voltage;
}

int Create::getCurrent()
{
	return current;
}

unsigned char Create::getBatteryTemperature()
{
	return batterytemperature;
}

unsigned char Create::getBatteryCharge()
{
	return batterycharge;
}

unsigned char Create::getBatteryCapacity()
{
	return batterycapacity;
}

int Create::getWallSignal()
{
	return wallsignal;
}

int Create::getCliffLS()
{
	return cliffls;
}

int Create::getCliffFLS()
{
	return clifffls;
}

int Create::getCliffFRS()
{
	return clifffrs;
}

int Create::getCliffRS()
{
	return cliffrs;
}

bool Create::getDigitalInput0()
{
	return digitalinput0;
}

bool Create::getDigitalInput1()
{
	return digitalinput1;
}

bool Create::getDigitalInput2()
{
	return digitalinput2;
}

bool Create::getDigitalInput3()
{
	return digitalinput3;
}

bool Create::getBaudRateChange() //DeviceDetect/BaudRateChange
{
	return baudchangerate;
}

int Create::getCargoAnalogSignal()
{
	return cargoanalogsignal;
}

bool Create::getHomeBase()
{
	return homebase;
}

bool Create::getInternalCharger()
{
	return internalcharger;
}

unsigned char Create::getOIMode()
{
	return oimode;
}

unsigned char Create::getSongNumber()
{
	return songnumber;
}

bool Create::getSongPlaying()
{
	return songplaying;
}

unsigned char Create::getStreamPackets()
{
	return streampackets;
}

int Create::getRequestedVelocity()
{
	return reqvelocity;
}

int Create::getRequestedRadius()
{
	return reqradius;
}

int Create::getRequestedRVelocity()
{
	return reqrvelocity;
}

int Create::getRequestedLVelocity()
{
	return reqlvelocity;
}

bool Create::getStreamingState()
{
	return streamingState;
}

//-------------------------------------
//Packet methods
//--------------------------------------

int Create::readBumpsAndWheelDrops(unsigned char *ins2)
{

	bumpRight = ins2[0] & 0x01;
	bumpLeft = ins2[0] & 0x02;
	wheelDropRight = ins2[0] & 0x04;
	wheelDropLeft = ins2[0] & 0x08;
	wheelDropCaster = ins2[0] & 0x10;
	
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readWall(unsigned char *ins2)
{
	wall = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readCliffLeft(unsigned char* ins2)
{
	cliffLeft = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readCliffFrontLeft(unsigned char*ins2)
{
	cliffFrontLeft = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readCliffFrontRight(unsigned char*ins2)
{
	cliffFrontRight = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readCliffRight(unsigned char*ins2)
{
	cliffRight = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readVirtualWall(unsigned char*ins2)
{
	virtualWall = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readLSDriverAndWheelO(unsigned char*ins2)
{	
	ld1 = ins2[0] & 0x01;
	ld0 = ins2[0] & 0x02;
	ld2 = ins2[0] & 0x04;
	rightWheel = ins2[0] & 0x08;
	leftWheel = ins2[0] & 0x10;
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

//Two unused bytes 15-16

int Create::readInfraredByte(unsigned char *ins2)
{
	infraredbyte = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readButtons(unsigned char *ins2)
{
	playbtn = ins2[0] & 0x01;
	advancebtn = ins2[0] & 0x04;
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readDistance(unsigned char*ins2)
{
	distance = 0;
	return toLittleEndian(ins2,2,&distance,SIGNED);
}

int Create::readAngle(unsigned char *ins2)
{
	angle = 0;
	return toLittleEndian(ins2,2,&angle,SIGNED);
}

int Create::readChargingState(unsigned char *ins2)
{
	chargingstate = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readVoltage(unsigned char *ins2)
{
	voltage = 0;
	return toLittleEndian(ins2,2,&voltage,UNSIGNED);
}

int Create::readCurrent(unsigned char *ins2)
{	current=0;
	return toLittleEndian(ins2,2,&current,SIGNED);
}

int Create::readBatteryTemperature(unsigned char *ins2)
{
	batterytemperature = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,SIGNED);
}

int Create::readBatteryCharge(unsigned char *ins2)
{
	batterycharge=0;
	return toLittleEndian(ins2,2,&batterycharge,UNSIGNED);
}

int Create::readBatteryCapacity(unsigned char *ins2)
{
	batterycapacity=0;
	return toLittleEndian(ins2,2,&batterycapacity,UNSIGNED);
}

int Create::readWallSignal(unsigned char *ins2)
{
	wallsignal=0;
	return toLittleEndian(ins2,2,&wallsignal,UNSIGNED);
}

int Create::readCliffLS(unsigned char *ins2)
{
	cliffls=0;
	return toLittleEndian(ins2,2,&cliffls,UNSIGNED);
}

int Create::readCliffFLS(unsigned char *ins2)
{
	clifffls=0;
	return toLittleEndian(ins2,2,&clifffls,UNSIGNED);
}

int Create::readCliffFRS(unsigned char *ins2)
{
	clifffrs=0;
	return toLittleEndian(ins2,2,&clifffrs,UNSIGNED);
}

int Create::readCliffRS(unsigned char *ins2)
{
	cliffrs=0;
	return toLittleEndian(ins2,2,&cliffrs,UNSIGNED);
}

int Create::readDigitalInputs(unsigned char *ins2)
{
	digitalinput0 = ins2[0] & 0x01;
	digitalinput1 = ins2[0] & 0x02;
	digitalinput2 = ins2[0] & 0x04;
	digitalinput3 = ins2[0] & 0x08;
	baudchangerate = ins2[0] & 0x10;
	int retValue;
	return toLittleEndian(ins2,1,&retValue,SIGNED);
}

int Create::readCargoAnalogSignal(unsigned char *ins2)
{
	cargoanalogsignal=0;
	return toLittleEndian(ins2,2,&cargoanalogsignal,UNSIGNED);
}

int Create::readChargingSources(unsigned char *ins2)
{
	internalcharger = ins2[0] & 0x01;
	homebase = ins2[0] & 0x02;
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readOIMode(unsigned char *ins2)
{
	oimode = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readSongNumber(unsigned char *ins2)
{
	songnumber = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readSongPlaying(unsigned char *ins2)
{
	songplaying = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readStreamPackets(unsigned char *ins2)
{
	streampackets = ins2[0];
	int retValue;
	return toLittleEndian(ins2,1,&retValue,UNSIGNED);
}

int Create::readReqVelocity(unsigned char *ins2)
{
	reqvelocity=0;
	return toLittleEndian(ins2,2,&reqvelocity,SIGNED);
}

int Create::readReqRadius(unsigned char *ins2)
{
	reqradius=0;
	return toLittleEndian(ins2,2,&reqradius,SIGNED);
}

int Create::readReqRVelocity(unsigned char *ins2)
{
	reqrvelocity=0;
	return toLittleEndian(ins2,2,&reqrvelocity,SIGNED);
}

int Create::readReqLVelocity(unsigned char *ins2)
{
	reqlvelocity=0;
	return toLittleEndian(ins2,2,&reqlvelocity,SIGNED);
}

int Create::updateSensor(unsigned char packetID, int instructionSize)
{	
	unsigned char *ins2 = new unsigned char[instructionSize];
	//usleep(120000);
	SleepMS(120);
	ReadFromSerial(portDescriptor,ins2,instructionSize);
	int ret = -65536;
	int offset=0;
	switch(packetID)
	{
		case 0: case 1: case 6: case 7: //packets 0,1,6 and 7 start here
			ret = readBumpsAndWheelDrops(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(7);
			}
			if(packetID == 7 )
				break;
		case 8:
			ret = readWall(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(8);
			}
			if(packetID == 8)
				break;
		case 9:
			ret = readCliffLeft(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(9);
			}
			if(packetID == 9)
				break;
		case 10:
			ret = readCliffFrontLeft(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(10);
			}
			if(packetID == 10)
				break;
		case 11:
			ret = readCliffFrontRight(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(11);
			}
			if(packetID == 11)
				break;
		case 12:
			ret = readCliffRight(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(12);
			}
			if(packetID == 12)
				break;
		case 13:
			ret = readVirtualWall(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(13);
			}
			if(packetID == 13)
				break;
		case 14:
			ret = readLSDriverAndWheelO(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(14);
			}
			if(packetID == 14)
				break;
		case 15:
		case 16:
			/*Unused bytes: Two unused bytes are sent after the overcurrent
			byte when the requested packet is 0, 1, or 6. The value of the
			two unused bytes is always 0.*/

		if(packetID < 7)
			{
				offset+= getSizePacket(16)+getSizePacket(15);
			}
			if(packetID == 15||packetID == 16||packetID == 1) //Ends packet 1
				break;
		case 2: case 17:                       //packet 2 and 17 start here
			ret = readInfraredByte(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(17);
			}			
			if(packetID == 17)
				break;
		case 18:
			ret = readButtons(ins2+offset);
		if(packetID < 7)
			{
				offset+= getSizePacket(18);
			}
			if(packetID == 18)
				break;
		case 19:
			ret = readDistance(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(19);
			}
			if(packetID == 19)
				break;
		case 20:
			ret = readAngle(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(20);
			}
			if(packetID == 20||packetID == 2) //Packet 2 ends here
				break;
		case 3:case 21:
			ret = readChargingState(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(21);
			}
			if(packetID == 21)
				break;
		case 22:
			ret = readVoltage(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(22);
			}
			if(packetID == 22)
				break;
		case 23:
			ret = readCurrent(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(23);
			}
			if(packetID == 23)
				break;
		case 24:
			ret = readBatteryTemperature(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(24);
			}
			if(packetID == 24)
				break;
		case 25:
			ret = readBatteryCharge(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(25);
			}
			if(packetID == 25)
				break;
		case 26:
			ret = readBatteryCapacity(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(26);
			}
			if(packetID == 26 ||packetID == 0 || packetID == 3) //Ends packets 0 and 3
				break;
		case 4: case 27:
			ret = readWallSignal(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(27);
			}
			if(packetID == 27)
				break;
		case 28:
			ret = readCliffLS(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(28);
			}
			if(packetID == 28)
				break;
		case 29:
			ret = readCliffFLS(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(29);
			}
			if(packetID == 29)
				break;
		case 30:
			ret = readCliffFRS(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(30);
			}
			if(packetID == 30)
				break;
		case 31:
			ret = readCliffRS(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(31);
			}
			if(packetID == 31)
				break;
		case 32:
			ret = readDigitalInputs(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(32);
			}
			if(packetID == 32)
				break;
		case 33:
			ret = readCargoAnalogSignal(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(33);
			}
			if(packetID == 33)
				break;
		case 34:
			ret = readChargingSources(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(34);
			}
			if(packetID == 34 || packetID == 4) //Ends packet 4
				break;
		case 5: case 35:	//Packet 5 starts
			ret = readOIMode(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(35);
			}
			if(packetID == 35)
				break;
		case 36:
			ret = readSongNumber(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(36);
			}
			if(packetID == 36)
				break;
		case 37:
			ret = readSongPlaying(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(37);
			}
			if(packetID == 37)
				break;
		case 38:
			ret = readStreamPackets(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(38);
			}
			if(packetID == 38)
				break;
		case 39:
			ret = readReqVelocity(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(39);
			}
			if(packetID == 39)
				break;
		case 40:
			ret = readReqRadius(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(40);
			}
			if(packetID == 40)
				break;
		case 41:
			ret = readReqRVelocity(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(41);
			}
			if(packetID == 41)
				break;
		case 42:
			ret = readReqLVelocity(ins2+offset);
			if(packetID < 7)
			{
				offset+= getSizePacket(42);
			}
			if(packetID == 42 ||packetID==5|| packetID == 6) //Ends packets 5 and 6
				break;
	}

	return ret;
}

int Create::getExternalSensors(int destinationArray[NUMBER_OF_SENSORS])
{
	GetSensors(destinationArray);
	return 1;
}

int Create::getExternalNthSensor(int n)
{
	return GetNthSensor(n);
}

bool Create::getExternalSensorsEnabledStatus()
{
	return externalSensorsEnabled;	
}

void Create::setVerbosity(t_verbosity level)
{
	SetSensorVerbosity( level);
	SetSerialPortVerbosity(level);
	robotVerbosity = level;
}
void Create::printRobotMessage(const char* message,...)
{
	if(robotVerbosity == VERBOSITY_NORMAL)
	{
		va_list arguments;
		va_start(arguments, message);
		vprintf(message,arguments);
		va_end(arguments);
	}
}



int Create::toLittleEndian( unsigned char* source,int nbytes,int* destination,boolSigned sign)
{
	char* tmp;	

	if(sign == UNSIGNED)
	{
	*destination =0;
	
	}
	else
	{
		if(source[0]&0x80) //if it is negative (2's complement)
		{
			*destination =-1;//0xFFFFFFF... only 1's
		}
		else
		{
			*destination = 0;
		}
	}
	
	tmp = (char*) destination;
	for(int i =nbytes -1; i>=0;i--)
	{
		*(tmp+((nbytes-1) - i)) = source[i];
		
	}	


return *destination;

}

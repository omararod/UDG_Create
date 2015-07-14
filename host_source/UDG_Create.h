#ifndef UDG_CREATE_H
#define UDG_CREATE_H

#include <Windows.h>
#include<string>
#include<sstream>
#include<vector>
#include<stdio.h>
#include <iostream>
#ifdef __linux
#include <unistd.h> 
#endif
#include <stdarg.h>
#include <string.h>
using namespace std;
#include <math.h>



enum errorCodes{INVALID_DEMO,INVALID_BAUDRATE,INVALID_INSTRUCTION_MODE,OUT_OF_RANGE,INVALID_MODE/*,EXTRENAL_SENSORS_ERROR*/};
enum modes{OFF,PASSIVE,SAFE,FULL};
enum baudCode{BAUD300,BAUD600,BAUD1200,BAUD2400,BAUD4800,BAUD9600,BAUD14400,BAUD19200,BAUD28800,BAUD38400,BAUD57600,BAUD115200};
enum chargingstates{NOT_CHARGING,RECONDITIONING_CHARGING,FULL_CHARGING,TRICKLE_CHARGING,WAITING,CHARGING_FAULT_CONDITION};
enum oimodes{OIOFF,OIPASSIVE,OISAFE,OIFULL};
enum infraredbytechars{IRLEFT=129,IRFORWARD,IRRIGHT,IRSPOT,IRMAX,IRSMALL,IRMEDIUM,IRLARGE,IRPAUSE,IRPOWER,
		       IRARC_FORWARD_LEFT,IRARC_FORWARD_RIGHT,IRDRIVE_STOP,IRSENDALL,IRSEEKDOCK,IRRESERVED,
		       IRRED,IRGREEN,IRFORCEFIELD,IRREDGREEN,IRREDFORCEFIELD,IRGREENFORCEFIELD,IRREDGREENFORCEFIELD};
enum sensorPackets{S_PACKET0,S_PACKET1,S_PACKET2,S_PACKET3,S_PACKET4,S_PACKET5,S_PACKET6,S_BUMPS_AND_WHEEL_DROPS ,S_WALL,S_CLIFF_LEFT,S_CLIFF_FRONT_LEFT,S_CLIFF_FRONT_RIGHT,S_CLIFF_RIGHT,S_VIRTUAL_WALL,S_OVERCURRENTS,S_UNUSED1,S_UNUSED2,S_IR_BYTE,S_BUTTONS,S_DISTANCE,S_ANGLE,S_CHARGING_STATE,S_VOLTAGE,S_CURRENT,S_BATTERY_TEMPREATURE,S_BATTERY_CHARGE,S_BATTERY_CAPACITY};

typedef enum VerbosityLevels {VERBOSITY_NORMAL,VERBOSITY_FILE,VERBOSITY_OFF, VERBOSITY_NUMBER_OF_LEVELS} t_verbosity;
typedef enum BoolSigned {SIGNED, UNSIGNED} boolSigned;
#define NUMBER_OF_SENSORS 14

typedef struct int_16
{
	unsigned char H; //High byte
	unsigned char L; //Low byte
} int16;



class Create
{
	public:
		Create();		
		Create(string);
		Create(string portName1,t_verbosity);
		~Create();
		string getPortName();
		bool isCharging();
		void start();
		void baud(unsigned char);
		void control();
		void safe();
		void full();
		void spot();
		void cover();
		void demo(unsigned char);
		void drive(int,int);
		void lowSideDrivers(unsigned char);
		void leds(unsigned char,unsigned char,unsigned char);
		void song(unsigned char,unsigned char,...);
		void playSong(unsigned char);
		int sensors(unsigned char);
		int getSizePacket(int);
		void coverAndDock();
		void pwmLowSideDrivers(unsigned char,unsigned char,unsigned char);
		void driveDirect(int,int);
		void digitalOutputs(unsigned char);
		//void stream(vector<unsigned char>);
		void stream(unsigned char* destinationBuffer,void* thread,int n,...);
		//void ThreadedReadStream(int,char*,int);
		void queryList(vector<unsigned char>);
		void pauseResumeStream(bool);
		void sendIr(unsigned char);
		void script(vector<unsigned char>);
		void script(unsigned char,...);
		void playScript();
		void showScript();
		void waitTime(unsigned char);
		void waitDistance(int);
		void waitAngle(int);
		void waitEvent(unsigned char);		
		void error(int,void*);
		int getBaudCode(int);
		char* charMode(int);	
		//void ThreadedReadStream(int portDescriptor,unsigned char*buffer,int numberOfBytes);
		////////////////
		//Get Prototypes
		////////////////
		bool getBumpRight();
		bool getBumpLeft();
		bool getWheelDropRight();
		bool getWheelDropLeft();
		bool getWheelDropCaster();
		bool getWallSeen();
		bool getCliffLeft();
		bool getCliffFrontLeft();
		bool getCliffFrontRight();
		bool getCliffRight();
		bool getVirtualWall();
		bool getLd0();
		bool getLd1();
		bool getLd2();
		bool getRightWheel();
		bool getLeftWheel();
		//Unused bytes 15-16.
		unsigned char getInfraredByte();
		bool getAdvanceBtn();
		bool getPlayBtn();
		int getDistance();
		int getAngle();
		unsigned char getChargingState();
		int getVoltage();
		int getCurrent();
		unsigned char getBatteryTemperature();
		unsigned char getBatteryCharge();
		unsigned char getBatteryCapacity();
		int getWallSignal();
		int getCliffLS();
		int getCliffFLS();
		int getCliffFRS();
		int getCliffRS();
		bool getDigitalInput0();
		bool getDigitalInput1();
		bool getDigitalInput2();
		bool getDigitalInput3();
		bool getBaudRateChange(); //DeviceDetect/BaudRateChange
		int getCargoAnalogSignal();
		bool getHomeBase();
		bool getInternalCharger();
		unsigned char getOIMode();
		unsigned char getSongNumber();
		bool getSongPlaying();
		unsigned char getStreamPackets();
		int getRequestedVelocity();
		int getRequestedRadius();
		int getRequestedRVelocity();
		int getRequestedLVelocity();
		bool getStreamingState();
		int getExternalSensors(int[NUMBER_OF_SENSORS]);
		int getExternalNthSensor(int);
		bool getExternalSensorsEnabledStatus();
		void setVerbosity(t_verbosity);
		void printRobotMessage(const char* message,...);
		int toLittleEndian(unsigned char* source,int nbytes,int* destination,boolSigned sign);
	private:
		/////////////////
		//Read Prototypes
		/////////////////
		int readBumpsAndWheelDrops(unsigned char *);
		int readWall(unsigned char *);
		int readCliffLeft(unsigned char *);
		int readCliffFrontLeft(unsigned char *);
		int readCliffFrontRight(unsigned char *);
		int readCliffRight(unsigned char *);
		int readVirtualWall(unsigned char *);
		int readLSDriverAndWheelO(unsigned char *);
		//Two unused bytes 15-16
		int readInfraredByte(unsigned char *);
		int readButtons(unsigned char *);
		int readDistance(unsigned char *);
		int readAngle(unsigned char *);
		int readChargingState(unsigned char *);
		int readVoltage(unsigned char *);
		int readCurrent(unsigned char *);
		int readBatteryTemperature(unsigned char *);
		int readBatteryCharge(unsigned char *);
		int readBatteryCapacity(unsigned char *);
		int readWallSignal(unsigned char *);
		int readCliffLS(unsigned char *);
		int readCliffFLS(unsigned char *);
		int readCliffFRS(unsigned char *);
		int readCliffRS(unsigned char *);
		int readDigitalInputs(unsigned char *);
		int readCargoAnalogSignal(unsigned char *);
		int readChargingSources(unsigned char *);
		int readOIMode(unsigned char *);
		int readSongNumber(unsigned char *);
		int readSongPlaying(unsigned char *);
		int readStreamPackets(unsigned char *);
		int readReqVelocity(unsigned char *);
		int readReqRadius(unsigned char *);
		int readReqRVelocity(unsigned char *);
		int readReqLVelocity(unsigned char *);
		int updateSensor(unsigned char, int);
		void commonInitializationProcedures(string,bool);
		string portName;
		int mode;
		//TODO: implement charging by hardware
		bool charging;
#ifndef __linux
		HANDLE portDescriptor;
#else
		int portDescriptor;
#endif
		int baudRate;
				
		bool bumpRight;
		bool bumpLeft;
		bool wheelDropRight;
		bool wheelDropLeft;
		bool wheelDropCaster;
		bool wall;
		bool cliffLeft;
		bool cliffFrontLeft;
		bool cliffFrontRight;
		bool cliffRight;
		bool virtualWall;
		bool ld0;
		bool ld1;
		bool ld2;
		bool rightWheel;
		bool leftWheel;
		//Unused bytes 15-16.
		unsigned char infraredbyte;
		bool advancebtn;
		bool playbtn;
		int distance;
		int angle;
		unsigned char chargingstate;
		int voltage;
		int current;
		unsigned char batterytemperature;
		int batterycharge;
		int batterycapacity;
		int wallsignal;
		int cliffls;
		int clifffls;
		int clifffrs;
		int cliffrs;
		bool digitalinput0;
		bool digitalinput1;
		bool digitalinput2;
		bool digitalinput3;
		bool baudchangerate;
		int cargoanalogsignal;
		bool homebase;
		bool internalcharger;
		unsigned char oimode;
		unsigned char songnumber;
		bool songplaying;
		unsigned char streampackets;
		int reqvelocity;
		int reqradius;
		int reqrvelocity;
		int reqlvelocity;
		bool streamingState;
		bool externalSensorsEnabled;
		t_verbosity robotVerbosity;
		int16 toInt16(int integer);
};

#endif

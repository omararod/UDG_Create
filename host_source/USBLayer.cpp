/*This is the host side for the microcontroller's USB firmware
This file is loosely based on "USB and PIC: quick guide to an USB HID framework"
by Alberto Maccioni, available at http://openprog.altervista.org/USB_firm_eng.html
*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/hiddev.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <fcntl.h>
#include "USBLayer.h"

#define BUFFER_SIZE 64
#define REPORT_SIZE 64
#define SLEEP_MILISECONDS 100

int fd = -1;
char devicePath[256];
unsigned char inBuffer[BUFFER_SIZE];
unsigned char outBuffer[BUFFER_SIZE];

//these structs hold information about the reports
struct hiddev_report_info inReportInfo;
struct hiddev_report_info outReportInfo;
//information about the endpoint usage
struct hiddev_usage_ref_multi inUsage;
struct hiddev_usage_ref_multi outUsage;

struct hiddev_devinfo deviceInfo;

t_verbosity USBVerbosity = VERBOSITY_NORMAL;

//this  section has to be re-implemented for windows***********************************************
int initializeUSB(int VID, int PID)
{	
	int i;
	int MAX_DESCRIPTORS = 50;
	printUSBMessage("Searching Create's Sensors USB Interface (VID 0x%04X and PID 0x%04X)\n",VID, PID);
	for( i = 0; i<MAX_DESCRIPTORS ;i++)
	{
		sprintf(devicePath,"/dev/usb/hiddev%d",i);
		fd = open(devicePath,O_RDONLY);
		if(fd >= 0)//if file was opened properly
		{
			ioctl(fd, HIDIOCGDEVINFO, &deviceInfo);
			if(deviceInfo.vendor == VID && deviceInfo.product == PID)
			{
				printf("Device found: %s\n",devicePath);
				break;
			}
		}
	}

	if(i >= MAX_DESCRIPTORS)
	{
		printUSBMessage("USB sensors not found!\n");
		return -1;
	}

	
	
	
	
}

void GetUSBData(unsigned char* destinationBuffer)
{	
	//Lots of info about the next part here http://www.wetlogic.net/hiddev/
	
	//Now that usb port is open, this is the actual initialization

	

	outReportInfo.report_type = HID_REPORT_TYPE_OUTPUT;
	outReportInfo.report_id = HID_REPORT_ID_FIRST;
	outReportInfo.num_fields = 1;

	inReportInfo.report_type = HID_REPORT_TYPE_INPUT;
	inReportInfo.report_id = HID_REPORT_ID_FIRST;
	inReportInfo.num_fields = 1;

	outUsage.uref.report_type = HID_REPORT_TYPE_OUTPUT;
	outUsage.uref.report_id = HID_REPORT_ID_FIRST;
	outUsage.uref.field_index = 0;
	outUsage.uref.usage_index = 0;
	outUsage.num_values = REPORT_SIZE;

	inUsage.uref.report_type = HID_REPORT_TYPE_INPUT;	
	inUsage.uref.report_id = HID_REPORT_ID_FIRST;
	inUsage.uref.field_index = 0;
	inUsage.uref.usage_index = 0;	
    	inUsage.num_values = REPORT_SIZE;

	/*
	HIDIOCSUSAGE: Sets the value of a usage in an output report.
	HIDIOCSREPORT: Instructs the kernel to send a report to the device. This report can be filled in by the user through HIDIOCSUSAGE calls (below) to fill in individual usage values in the report before sending the report in full to the device.


	HIDIOCSUSAGE:Returns the value of a usage in a hiddev_usage_ref structure. The usage to be retrieved can be specified as above, or the user can choose to fill in the report_type field and specify the report_id as HID_REPORT_ID_UNKNOWN. In this case, the hiddev_usage_ref will be filled in with the report and field infomation associated with this usage if it is found.
	HIDIOCGREPORTInstructs the kernel to get a feature or input report from the device, in order to selectively update the usage structures (in contrast to INITREPORT).

	*/
	
	outUsage.values[0] = 0x37;//GET_SENSORS_REPORT_CODE;//0x37 is the report code to read all of the PIC's ADC's
	
	//write
	ioctl(fd,HIDIOCSUSAGES, &outUsage); //set the output report
	ioctl(fd,HIDIOCSREPORT, &outReportInfo); //send the output report
	usleep(SLEEP_MILISECONDS*1000);
	//read
	ioctl(fd,HIDIOCGUSAGES, &inUsage); //set the input report
	ioctl(fd,HIDIOCGREPORT, &inReportInfo); //get the input report
	
	//printf("-->");
	for(int i = 1 ; i < REPORT_SIZE ; i++)
	{destinationBuffer[i] = inUsage.values[i];}

	//printf("--%X--",destinationBuffer[28]);

	
}

void CloseUSB()
{
	close(fd);
}

//*************************************************************************************************************
void printUSBMessage(const char* message,...)
{
	if(USBVerbosity == VERBOSITY_NORMAL)
	{
		va_list arguments;
		va_start(arguments, message);
		vprintf(message,arguments);
		va_end(arguments);
	}
}

void SetUSBVerbosity(t_verbosity level)
{
	if(level < VERBOSITY_NUMBER_OF_LEVELS)
	{
		USBVerbosity = level;
	}
}

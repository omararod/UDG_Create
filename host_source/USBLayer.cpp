/*This is the host side for the microcontroller's USB firmware
The linux part of this file is loosely based on "USB and PIC: quick guide to an USB HID framework"
by Alberto Maccioni, available at http://openprog.altervista.org/USB_firm_eng.html
*/

#include "USBLayer.h"
#define SLEEP_MILISECONDS 100
#ifdef __linux
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/hiddev.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <fcntl.h>


#define BUFFER_SIZE 64
#define REPORT_SIZE 64


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

#else
//Global declarations
HANDLE USBWriteHandle = INVALID_HANDLE_VALUE;
HANDLE USBReadHandle = INVALID_HANDLE_VALUE;
// GUID for HID class
GUID HIDClassGuid = { 0x4d1e55b2, 0xf16f, 0x11cf, 0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30 };

#endif

t_verbosity USBVerbosity = VERBOSITY_NORMAL;

//this  section has to be re-implemented for windows***********************************************
int initializeUSB(int VID, int PID)
{

#ifdef __linux
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
				printUSBMessage("Device found: %s\n",devicePath);
				break;
			}
		}
	}

	if(i >= MAX_DESCRIPTORS)
	{
		printUSBMessage("USB sensors not found!\n");
		return -1;
	}
#else
	//register for WM_DEVICECHANGE message
	DEV_BROADCAST_DEVICEINTERFACE broadcastDescriptor;
	broadcastDescriptor.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
	broadcastDescriptor.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
	broadcastDescriptor.dbcc_reserved = 0;
	broadcastDescriptor.dbcc_classguid = HIDClassGuid;

	RegisterDeviceNotification(NULL, &broadcastDescriptor, DEVICE_NOTIFY_WINDOW_HANDLE);


	//local declarations
	SP_DEVICE_INTERFACE_DATA deviceData;
	SP_DEVINFO_DATA deviceInfo;
	PSP_DEVICE_INTERFACE_DETAIL_DATA deviceInterfaceDetail;
	unsigned int index = 0;
	unsigned int lastError;
	DWORD registryType = 0;
	DWORD registrySize = 0;
	DWORD registrySize2 = 0;
	DWORD interfaceInfoSize;
	byte* buffer;
	wchar_t tempString[512];

	//Enumerate devices from HID class
	//DIGCF_PRESENT Return only devices that are currently present in a system.
	//DIGCF_DEVICEINTERFACEReturn devices that support device interfaces for the specified device interface classes
	HDEVINFO deviceTable = SetupDiGetClassDevs(&HIDClassGuid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	if (deviceTable == INVALID_HANDLE_VALUE)
	{
		printUSBMessage("Error while filling device table  from SetupDiGetClassDevs()\n");
		return -1;
	}

	deviceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
	//look through the list for the device. We assume there will not be more than 1000 devices.
	while (index < 1000)
	{
		if (!SetupDiEnumDeviceInterfaces(deviceTable, NULL, &HIDClassGuid, index, &deviceData))
		{
			lastError = GetLastError();
			if (lastError == ERROR_NO_MORE_ITEMS)
			{
				SetupDiDestroyDeviceInfoList(&deviceTable);
				break;
			}
			else
			{
				SetupDiDestroyDeviceInfoList(&deviceTable);
				printUSBMessage("something went wrong when looking for the device\n");
				return -1;
			}
		}
		//get device info ID string
		deviceInfo.cbSize = sizeof(SP_DEVINFO_DATA);
		SetupDiEnumDeviceInfo(deviceTable, index, &deviceInfo);
		//we call this function twice, one to get the necessary buffer size and the second to get the actual ID info
		SetupDiGetDeviceRegistryProperty(deviceTable, &deviceInfo, SPDRP_HARDWAREID, &registryType, NULL, 0, &registrySize);
		buffer = new byte[registrySize];
		SetupDiGetDeviceRegistryProperty(deviceTable, &deviceInfo, SPDRP_HARDWAREID, &registryType, buffer, registrySize, &registrySize2);
		lstrcpy(tempString, (LPCWSTR)buffer);
		free(buffer);
		//we use substring because the device is actually enumerated as HID\\VID_04D8&PID_003F&REV_XXXX
		if (lstrcmp(wstring(tempString).substr(0, 21).c_str(), VID_PID) == 0)
		{ //device is "HID\\VID_04D8&PID_003F"

			//We also call this function twice, first to get the size and then the actual info
			SetupDiGetDeviceInterfaceDetail(deviceTable, &deviceData, NULL, 0, &interfaceInfoSize, NULL);

			deviceInterfaceDetail = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(interfaceInfoSize);
			if (deviceInterfaceDetail)
			{
				deviceInterfaceDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
				ZeroMemory(deviceInterfaceDetail->DevicePath, sizeof(deviceInterfaceDetail->DevicePath));
			}
			if (SetupDiGetDeviceInterfaceDetail(deviceTable, &deviceData, deviceInterfaceDetail, interfaceInfoSize, NULL, NULL))
			{
				SetupDiDestroyDeviceInfoList(deviceTable);
				USBReadHandle = CreateFile(deviceInterfaceDetail->DevicePath, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
				if (USBReadHandle == INVALID_HANDLE_VALUE)
				{
					printUSBMessage("Error opening usb reading handle. Closing...\n");
					return -1;
				}
				USBWriteHandle = CreateFile(deviceInterfaceDetail->DevicePath, GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
				if (!USBWriteHandle || USBWriteHandle == INVALID_HANDLE_VALUE)
				{
					printUSBMessage("Error opening usb write handle. Closing...\n");
					return -1;
				}

				break;
			}
		}
		index++;
	}

#endif
	
	
	
	return 1;
}

void GetUSBData(unsigned char* destinationBuffer)
{	
#ifdef __linux
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

#else
	byte buffer[65];
	DWORD bytesWritten;
	buffer[0] = 0x0;  //read sensors
	buffer[1] = 0x37;  //read sensors
	WriteFile(USBWriteHandle, buffer, 65, &bytesWritten, NULL);
	//if no bytes were written then something went wrong
	if (bytesWritten <= 0)
		printUSBMessage("Error writing to USB\n");

	Sleep(SLEEP_MILISECONDS);

	DWORD readBytes;
	ReadFile(USBReadHandle, destinationBuffer, 65, &readBytes, NULL);
	//if no bytes were read then something went wrong
	if (readBytes <= 0)
		printUSBMessage("Error reading from USB\n");

#endif

	
}

void CloseUSB()
{
#ifdef __linux
	close(fd);
#else
	if (USBWriteHandle)
	CloseHandle(USBWriteHandle);
	if (USBReadHandle)
	CloseHandle(USBReadHandle);
#endif
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

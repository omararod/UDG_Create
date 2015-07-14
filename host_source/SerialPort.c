/*Author: Omar Alejandro Rodríguez Rosas

based on the examples at http://en.wikibooks.org/wiki/Serial_Programming/termios
*/
#include "SerialPort.h"



t_verbosity SerialPortVerbosity;

/*-----------------------------------------------------------------------------------------
string PrintError(int errorCode)
errorCode: an "Error" enumeration member that represents a particular erro condition

Prints a message decribing a particular error code 
--------------------------------------------------------------------------------------------
*/
void PrintError(int errorCode)
{
	if(SerialPortVerbosity == VERBOSITY_NORMAL)
	{
	printSerialMessage("E%i :",errorCode);
	
	switch(errorCode)
		{
			case ERROR_OPEN:
			printSerialMessage(" An error has occurred while trying to open a port.\n");
			break;
			
			case ERROR_SET_SPEED:
			printSerialMessage(" An error has occurred while setting the serial port baud rate\n");
			break;
			
			case ERROR_CONFIGURE_PORT:
			printSerialMessage(" An error has occurred while setting the serial port configuration\n");
			break;
	
			case ERROR_WRITE:
			printSerialMessage(" An error has occurred while writing to the serial port\n");
			break;
	
			case ERROR_READ:
			printSerialMessage(" An error has occurred while reading from the serial port\n");
			break;
		}
	}
	
}
#ifndef __linux
HANDLE OpenPort(LPWSTR portName)
{
	HANDLE portDescriptor = CreateFile(portName,
		GENERIC_READ | GENERIC_WRITE,
		0,0,OPEN_EXISTING,0,0);
	if(portDescriptor == INVALID_HANDLE_VALUE)
	{
		printSerialMessage("Failed opening %ls\n",portName);
		return INVALID_HANDLE_VALUE;
	}
	DCB deviceControlBlock;
	ZeroMemory(&deviceControlBlock, sizeof(deviceControlBlock));
	deviceControlBlock.DCBlength = sizeof(DCB);
	BuildCommDCB(TEXT("57600,n,8,1"), &deviceControlBlock);
	
	if (!SetCommState(portDescriptor,&deviceControlBlock))
	{
		printSerialMessage("Error configuring port\n");
		return INVALID_HANDLE_VALUE;
	}
	else
	{
		printSerialMessage("Succesfully opened %s\n", portName);
    }
	return portDescriptor;
}

HANDLE AutoOpenPort()
{
	wstring portPrefix(L"COM");
	HANDLE portDescriptor = INVALID_HANDLE_VALUE;
	wstring portName;
	for(int i = 1; i< 10; i++)
	{
		portName = portPrefix + to_wstring(i);
		portDescriptor = OpenPort((LPWSTR)portName.c_str());
		printSerialMessage("Trying to open %ls\n", portName.c_str());
		if (portDescriptor != INVALID_HANDLE_VALUE)
		{
			printSerialMessage("Succesfully opened %ls\n", portName.c_str());
			return portDescriptor;
		}
	}



	portPrefix = L"\\\\.\\COM";
	for (int i = 10; i< 21; i++)
	{
		portName = portPrefix + to_wstring(i);
		portDescriptor = OpenPort((LPWSTR)portName.c_str());
		printSerialMessage("Trying to open %ls\n", portName.c_str());
		if (portDescriptor != INVALID_HANDLE_VALUE)
		{
			printSerialMessage("Succesfully opened %ls\n", portName.c_str());
			return portDescriptor;
		}
	}

	printSerialMessage("No serial port could be opened \n");
	return portDescriptor;

}

/*-----------------------------------------------------------------------------------------
int WriteToSerial(HANDLE portDescriptor,char* buffer, int numberOfBytes)

writes numberOfBytes bytes from buffer to portDescriptor port
--------------------------------------------------------------------------------------------
*/
int WriteToSerial(HANDLE portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	DWORD writtenBytes = 0;
	if (portDescriptor != INVALID_HANDLE_VALUE)
		WriteFile(portDescriptor, buffer, numberOfBytes, &writtenBytes, NULL);
	return (int)(writtenBytes);
}

int ReadFromSerial(HANDLE portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	DWORD readBytes = 0;
	if (portDescriptor != INVALID_HANDLE_VALUE)
		ReadFile(portDescriptor, buffer, numberOfBytes, &readBytes, NULL);
	return (int)(readBytes);
}

void StartThreadedRead(HANDLE portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	std::thread t(ThreadedRead, portDescriptor, buffer, numberOfBytes);
	t.join();

}

void ThreadedRead(HANDLE portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	while (1)
	{
		if (ReadFile(portDescriptor, buffer, numberOfBytes, NULL, NULL))
			cout << *buffer;
	}
}

#else
/*-----------------------------------------------------------------------------------------
int OpenPort(const char* PortName)
Receives a char string rpresenting the name of the port to open (i.e. "/dev/tty0")
and returns an int descriptor of such port. Returns -1 if the port can't be opened.
Additional info:
O_RDWR: Opens the port for reading and writing
O_NOCTTY:The port never becomes the controlling terminal of the process.
O_NDELAY:Use non-blocking I/O. On some systems this also means the RS232 DCD signal line is ignored.
--------------------------------------------------------------------------------------------
*/
int OpenPort(const char* portName)
{   
	
	int portDescriptor = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
	
	if(portDescriptor<0) 
	{  	PrintError(ERROR_OPEN);
		printSerialMessage("Port: %s\n",portName);
	}
	else
	{
		printSerialMessage("Succesfully opened %s\n",portName);
	}
	
	return portDescriptor;
}
int AutoOpenPort()
{
	string portPrefix("/dev/ttyUSB");
	int portDescriptor = -1;
	string portName;
	for (int i = 0; i< 10; i++)
	{
		portName = portPrefix + to_string(i);
		portDescriptor = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		printSerialMessage("Trying to open %s\n", portName.c_str());
		if (portDescriptor > 0)
		{
			printSerialMessage("Succesfully opened %s\n", portName.c_str());
			return portDescriptor;
		}
	}



	portPrefix = "/dev/ttyS";
	for (int i = 0; i< 50; i++)
	{
		portName = portPrefix + to_string(i);
		portDescriptor = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		printSerialMessage("Trying to open %s\n", portName.c_str());
		if (portDescriptor > 0)
		{
			printSerialMessage("Succesfully opened %s\n", portName.c_str());
			return portDescriptor;
		}
	}

	printSerialMessage("No serial port could be opened \n");
	return portDescriptor;

}

/*-----------------------------------------------------------------------------------------
int ConfigurePort(int portDescriptor)
PortDescriptor: An int descriptor for the selected port

Sets all the needed configurations for serial communication. Returns -1 if something
goes wrong

--------------------------------------------------------------------------------------------
*/
int ConfigurePort(int portDescriptor)
{

	int returnValue = 1;

	struct termios configuration;
	//===================Configure port===================
	//more info http://homepages.cwi.nl/~aeb/linux/man2html/man3/termios.3.html
	/*IGNBRK:	Ignore break conditions
	BRKINT:	flushes the queue when a brake is received
	ICRNL:	Translate CR to NL
	INLCR:	Translate NL to CR
	PARMRK:	Mark parity errors
	INPCK:	Parity checking
	ISTRIP:	Strip off eight bit
	IXON:	XON/XOFF flow control
	*/
	//Disables those options
	memset(&configuration, 0, sizeof(configuration));
	configuration.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
		INLCR | PARMRK | INPCK | ISTRIP | IXON);
	configuration.c_oflag = 0;
	/*ECHO:	echo input characters
	ECHONL: echoes the NL characer if ICANON is set
	ICANON: Canonical mode
	IEXTEN: inplementation-defined input processing
	ISIG:	generate INTR, QUIT, SUSP, or DSUSP signals
	*/
	configuration.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	/*CSIZE: Character size mask. Values are CS5, CS6, CS7, or CS8
	PARENB: parity generation on output and parity checking for input.
	CS8: See CSIZE
	*/
	configuration.c_cflag &= ~(CSIZE | PARENB);
	configuration.c_cflag |= CS8;
	/*VMIN: minimum number of bytes received before read() returns
	VTIME: Tenths of second before consider a read() as finished
	*/
	configuration.c_cc[VMIN] = 1;
	configuration.c_cc[VTIME] = 0;

	returnValue = cfsetispeed(&configuration, B57600) < 0 || cfsetospeed(&configuration, B57600);
	if (returnValue<0)
	{
		PrintError(ERROR_SET_SPEED);
		return returnValue;
	}
	/*TCSANOW: Apply immediately
	*/
	returnValue = tcsetattr(portDescriptor, TCSANOW, &configuration);
	if (returnValue < 0)
	{
		PrintError(ERROR_CONFIGURE_PORT);
	}
	printSerialMessage("Serial Port Succesfully configured\n");
	return returnValue;
}

/*-----------------------------------------------------------------------------------------
int WriteToSerial(int portDescriptor,char* buffer, int numberOfBytes)

writes numberOfBytes bytes from buffer to portDescriptor port
--------------------------------------------------------------------------------------------
*/
int WriteToSerial(int portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	int returnValue;
	returnValue = write(portDescriptor, buffer, numberOfBytes);
	if (returnValue<0)
	{
		PrintError(ERROR_WRITE);
	}
	return returnValue;
}

int ReadFromSerial(int portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	int returnValue;
	returnValue = read(portDescriptor, buffer, numberOfBytes);
	if (returnValue<0)
	{
		PrintError(ERROR_READ);
	}
	return returnValue;
}

void StartThreadedRead(int portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	std::thread t(ThreadedRead, portDescriptor, buffer, numberOfBytes);
	t.join();

}

void ThreadedRead(int portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	while (1)
	{
		if (read(portDescriptor, buffer, numberOfBytes)>0)
			cout << *buffer;
	}
}

void StartThreadedRead(int portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	std::thread t(ThreadedRead, portDescriptor, buffer, numberOfBytes);
	t.join();

}

void ThreadedRead(int portDescriptor, unsigned char* buffer, int numberOfBytes)
{
	while (1)
	{
		if (ReadFile(portDescriptor, buffer, numberOfBytes, NULL, NULL))
			cout << *buffer;
	}
}
#endif




void printSerialMessage(const char* message,...)
{
	if(SerialPortVerbosity == VERBOSITY_NORMAL)
	{
		va_list arguments;
		va_start(arguments, message);
		vprintf(message,arguments);
		va_end(arguments);
	}
}

void SetSerialPortVerbosity(t_verbosity level)
{
	SerialPortVerbosity = level;
}

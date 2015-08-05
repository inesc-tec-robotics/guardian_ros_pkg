/** \file RoboteqDevice.cpp
 * \version 080411
 * \date    2011
 *
 * \brief class for RoboteqDevice servo driver
 * (C) Roboteq Inc., 2012
*/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sstream>

#include <ros/console.h>

#include "guardian_node/RoboteqDevice.h"
#include "guardian_node/ErrorCodes.h"

using namespace std;

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

RoboteqDevice::RoboteqDevice() : device_fd(0), fd0(0)
{
	handle = RQ_INVALID_HANDLE;
}
RoboteqDevice::~RoboteqDevice()
{
	Disconnect();
}

bool RoboteqDevice::IsConnected()
{
	return handle != RQ_INVALID_HANDLE;
}
int RoboteqDevice::Connect(string port)
{
	if(IsConnected())
	{
		cout<<"RoboteqDevice::Connect: Device is connected, attempting to disconnect."<<endl;
		Disconnect();
	}

	//Open port.
	cout<<"RoboteqDevice::Connect: Opening port '"<<port<<"'...";
	handle = open(port.c_str(), O_RDWR |O_NOCTTY | O_NDELAY); // Original
	//handle = open(port.c_str(), O_RDWR |O_NOCTTY); // Adapted by Robotnik
	if(handle == RQ_INVALID_HANDLE)
	{
		cout<<"failed."<<endl;
		return RQ_ERR_OPEN_PORT;
	}

	cout<<"...succeeded."<<endl;
	fcntl (handle, F_SETFL, O_APPEND | O_NONBLOCK); //original

	cout<<"RoboteqDevice::Connect: Initializing port...";
	InitPort();
	cout<<"...done."<<endl;

	int status;
	string response;
	cout<<"Detecting device version...";
	status = IssueCommand("?", "$1E", 10, response);
	if(status != RQ_SUCCESS)
	{
		cout<<"RoboteqDevice::Connect: failed (issue ?$1E response: "<<status<<")."<<endl;
		Disconnect();
		return RQ_UNRECOGNIZED_DEVICE;
	}

	if(response.length() < 12)
	{
		cout<<"RoboteqDevice::Connect: failed (unrecognized version)."<<endl;
		Disconnect();
		return RQ_UNRECOGNIZED_VERSION;
	}

	cout<<response.substr(8, 4)<<"."<<endl;
	return RQ_SUCCESS;
}
void RoboteqDevice::Disconnect()
{
	if(!IsConnected())
		close(handle);

	handle = RQ_INVALID_HANDLE;
}

void RoboteqDevice::InitPort()
{
	if(!IsConnected())
		return;

	//Get the existing Comm Port Attributes in cwrget
	struct termios newtio;
	tcgetattr (handle, &newtio);

	//Set the Tx and Rx Baud Rate to 9600
	if(cfsetospeed (&newtio, B115200) < 0)
		cout << "RoboteqDevice::InitPort: Error in cfsetospeed " << endl;
	if(cfsetispeed (&newtio, B115200) < 0)
		cout << "RoboteqDevice::InitPort: Error in cfsetispeed " << endl;

	newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~HUPCL;
	//parity = NONE
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~PARODD;

	newtio.c_cflag &= ~CSTOPB;// 1 Stop bit
	newtio.c_cflag &= ~CSIZE;

	newtio.c_cflag |= CS8;

	newtio.c_cflag &= ~CRTSCTS;	//Disables hardware flow control

    newtio.c_iflag |= IGNBRK;
    newtio.c_iflag &= ~ICRNL;
    newtio.c_iflag &= ~IXON;
    newtio.c_oflag &= ~OPOST;
    newtio.c_oflag &= ~ONLCR;
    newtio.c_lflag &= ~ISIG;
    newtio.c_lflag &= ~IEXTEN;
    newtio.c_lflag &= ~ECHOK;
    newtio.c_lflag &= ~ECHOCTL;
    newtio.c_lflag &= ~ECHOKE;
    newtio.c_lflag &= ~ECHO;
    newtio.c_lflag &= ~ECHOE;
    //
    // Entrada canónica-> La entrada canónica es orientada a línea. Los caracteres se meten en un buffer hasta recibir un CR o LF.
    newtio.c_lflag &= ~ICANON;

    // carácteres de control
    //options.c_cc[VMIN] = (cc_t)1;
    newtio.c_cc[VMIN] = (cc_t)1;
    newtio.c_cc[VTIME] = (cc_t)5;

    tcflush(handle, TCIFLUSH);

	
	//Enable the Receiver and  Set local Mode
	//newtio.c_iflag = IGNBRK;		/* Ignore Break Condition & no processing under input options*/
	//newtio.c_lflag = 0;			/* Select the RAW Input Mode through Local options*/
	//newtio.c_oflag = 0;			/* Select the RAW Output Mode through Local options*/
	//newtio.c_cflag |= (CLOCAL | CREAD);	/* Select the Local Mode & Enable Receiver through Control options*/
	/// TEST
	//newtio.c_cflag &= ~HUPCL;	
	
	//Set Data format to 7E1
	//newtio.c_cflag &= ~CSIZE;		/* Mask the Character Size Bits through Control options*/
	//newtio.c_cflag |= CS7;			/* Select Character Size to 7-Bits through Control options*/
	//newtio.c_cflag |= PARENB;		/* Select the Parity Enable through Control options*/
	//newtio.c_cflag &= ~PARODD;		/* Select the Even Parity through Control options*/
	/// TEST
	//newtio.c_cflag &= ~CSTOPB;// 1 Stop bit

	//cwrset.c_iflag |= (INPCK|ISTRIP);
	//cwrset.c_cc[VMIN] = 6;

	/// TEST
	//newtio.c_cc[VMIN] = (cc_t)1;
    //newtio.c_cc[VTIME] = (cc_t)5; /// TEST


	//Set the New Comm Port Attributes through cwrset
	//tcsetattr (fd0, TCSANOW, &newtio);	/* Set the attribute NOW without waiting for Data to Complete*/

	/// TEST
	tcsetattr (handle, TCSANOW, &newtio);

    fcntl(handle,F_SETFL, FNDELAY);	
}

int RoboteqDevice::Write(string str, double timeout_millisec)
{
	ROS_DEBUG_STREAM_NAMED("serial_data_write", str);

	if(!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	//cout<<"Writing: "<<ReplaceString(str, "\r", "\r\n");
	size_t number_bytes_left = str.length();
	size_t current_byte = 0;
	double start_time = get_current_time_ms();
	while (number_bytes_left > 0) {
		if (get_current_time_ms() - start_time > timeout_millisec) {
			ROS_DEBUG_STREAM_NAMED("serial_data_write", "Timeout of " << timeout_millisec << "reached when sending " << str.length() << " bytes");
			return RQ_ERR_TRANSMIT_FAILED;
		}

		int number_bytes_sent = write(handle, &str.c_str()[current_byte], number_bytes_left);
		if (number_bytes_sent < 0) {
			return RQ_ERR_TRANSMIT_FAILED;
		}
		
		number_bytes_left -= number_bytes_sent;
		current_byte += number_bytes_sent;
	}

	if (fsync(handle) < 0) {
		if (errno != EROFS && errno != EINVAL) { return RQ_ERR_TRANSMIT_FAILED; }
	}

	return RQ_SUCCESS;
}

int RoboteqDevice::ReadAll(string &str, size_t number_of_expected_lines, double timeout_millisec, char line_delimiter)
{
	if(!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	char buf[BUFFER_SIZE + 1] = "";

	str = "";
	int countRcv = 0;
	size_t number_of_lines_received = 0;
	double start_time = get_current_time_ms();
	while(number_of_lines_received < number_of_expected_lines)
	{
		if (get_current_time_ms() - start_time > timeout_millisec) {
			ROS_DEBUG_STREAM_NAMED("serial_data_read", "Timeout of " << timeout_millisec << " ms reached: read " << number_of_lines_received << " lines with " << str.size() << " bytes -> content: [" << str << "]");
			return RQ_ERR_SERIAL_IO;
		}

		countRcv = read(handle, buf, BUFFER_SIZE);
		if(countRcv == -1 && errno != EAGAIN) { return RQ_ERR_SERIAL_RECEIVE; }

		if(countRcv > (BUFFER_SIZE + 1)) {
			ROS_DEBUG_STREAM_NAMED("serial_data_read", "Received " << countRcv << " bytes with a buffer of " << (BUFFER_SIZE + 1));
			return RQ_ERR_SERIAL_RECEIVE;
		}

		if (countRcv > 0) {
			ROS_DEBUG_STREAM_NAMED("serial_data_read", "Received " << countRcv << " bytes -> content: [" << std::string(buf, countRcv) << "]" );
			for (int i = 0; i < countRcv; ++i) {
				if (buf[i] == line_delimiter) { ++number_of_lines_received; }
			}
			str.append(buf, countRcv);
		}
	}

    ROS_DEBUG_STREAM_NAMED("serial_data_read", str);

	return RQ_SUCCESS;
}

int RoboteqDevice::IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus, size_t number_of_expected_lines, double timeout_millisec)
{
	int status;
	string read;
	response = "";

	if(args == "")
		status = Write(commandType + command + "\r", timeout_millisec);
	else
		status = Write(commandType + command + " " + args + "\r", timeout_millisec);

	if(status != RQ_SUCCESS) {
		cout << "RoboteqDevice::IssueCommand: error Writing" << endl;
		return status;
	}

	usleep(waitms * 1000l);

	status = ReadAll(read, number_of_expected_lines, timeout_millisec);
	if(status != RQ_SUCCESS) {
		cout << "RoboteqDevice::IssueCommand: error Reading status = " << status << endl;
		return status;
	}

	if(isplusminus) {
		if(read.length() < 2)
			return RQ_INVALID_RESPONSE;

		response = read.substr(read.length() - 2, 1);
		return RQ_SUCCESS;
	}


	string::size_type pos = read.rfind(command + "=");
	if(pos == string::npos)
		return RQ_INVALID_RESPONSE;

	pos += command.length() + 1;

	string::size_type carriage = read.find("\r", pos);
	if(carriage == string::npos)
		return RQ_INVALID_RESPONSE;

	response = read.substr(pos, carriage - pos);

	return RQ_SUCCESS;
}
int RoboteqDevice::IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus, size_t number_of_expected_lines, double timeout_millisec)
{
	return IssueCommand(commandType, command, "", waitms, response, isplusminus, number_of_expected_lines, timeout_millisec);
}

int RoboteqDevice::SetConfig(int configItem, int index, int value)
{
	string response;
	char command[10];
	char args[50];

	if(configItem < 0 || configItem > 255)
		return RQ_INVALID_CONFIG_ITEM;

	sprintf(command, "$%02X", configItem);
	sprintf(args, "%i %i", index, value);
	if(index == MISSING_VALUE)
	{
		sprintf(args, "%i", value);
		index = 0;
	}

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	int status = IssueCommand("^", command, args, 10, response, true);
	if(status != RQ_SUCCESS)
		return status;
	if(response != "+")
		return RQ_SET_CONFIG_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::SetConfig(int configItem, int value)
{
	return SetConfig(configItem, MISSING_VALUE, value);
}

int RoboteqDevice::SetCommand(int commandItem, int index, int value)
{
	string response;
	char command[10];
	char args[50];

	if(commandItem < 0 || commandItem > 255)
		return RQ_INVALID_COMMAND_ITEM;

	sprintf(command, "$%02X", commandItem);
	sprintf(args, "%i %i", index, value);
	if(index == MISSING_VALUE)
	{
		if(value != MISSING_VALUE)
			sprintf(args, "%i", value);
		index = 0;
	}

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	int status = IssueCommand("!", command, args, 10, response, true);
	if(status != RQ_SUCCESS)
		return status;
	if(response != "+")
		return RQ_SET_COMMAND_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::SetCommand(int commandItem, int value)
{
	return SetCommand(commandItem, MISSING_VALUE, value);
}
int RoboteqDevice::SetCommand(int commandItem)
{
	return SetCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
}

int RoboteqDevice::GetConfig(int configItem, int index, int &result)
{
	string response;
	char command[10];
	char args[50];

	if(configItem < 0 || configItem > 255)
		return RQ_INVALID_CONFIG_ITEM;

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	sprintf(command, "$%02X", configItem);
	sprintf(args, "%i", index);

	int status = IssueCommand("~", command, args, 10, response);
	if(status != RQ_SUCCESS)
		return status;

	istringstream iss(response);
	iss>>result;

	if(iss.fail())
		return RQ_GET_CONFIG_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::GetConfig(int configItem, int &result)
{
	return GetConfig(configItem, 0, result);
}

int RoboteqDevice::GetValue(int operatingItem, int index, int &result)
{
	string response;
	char command[10];
	char args[50];

	if(operatingItem < 0 || operatingItem > 255)
		return RQ_INVALID_OPER_ITEM;

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	sprintf(command, "$%02X", operatingItem);
	sprintf(args, "%i", index);

	int status = IssueCommand("?", command, args, 10, response);
	if(status != RQ_SUCCESS)
		return status;

	istringstream iss(response);
	iss>>result;

	if(iss.fail())
		return RQ_GET_VALUE_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::GetValue(int operatingItem, int &result)
{
	return GetValue(operatingItem, 0, result);
}


string ReplaceString(string source, string find, string replacement)
{
	string::size_type pos = 0;
    while((pos = source.find(find, pos)) != string::npos)
	{
        source.replace(pos, find.size(), replacement);
        pos++;
    }

	return source;
}

void sleepms(int milliseconds)
{
	usleep(milliseconds / 1000);
}

double get_current_time_ms() { //returns current time in seconds
	timeval tv;
	gettimeofday(&tv, NULL);
	double rtn_value = (double) tv.tv_usec;
	rtn_value /= 1e6;
	rtn_value += (double) tv.tv_sec;
	return rtn_value * 1000.0;
}

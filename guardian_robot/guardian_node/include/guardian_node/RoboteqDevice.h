/** \file RoboteqDevice.h
 * \version 080411
 * \date    2011
 *
 * \brief class for RoboteqDevice servo driver
 * (C) Roboteq Inc., 2012
*/

#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

#include <time.h>

using namespace std;

string ReplaceString(string source, string find, string replacement);
void sleepms(int milliseconds);
double get_current_time_ms();

class RoboteqDevice
{
private:
	int device_fd;
	int fd0;
	int handle;

protected:
	void InitPort();

	int Write(string str, double timeout_millisec = 500);
	int ReadAll(string &str, size_t number_of_expected_lines = 2, double timeout_millisec = 500, char line_delimiter = '\r');

	int IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus = false, size_t number_of_expected_lines = 2, double timeout_millisec = 500);
	int IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus = false, size_t number_of_expected_lines = 2, double timeout_millisec = 500);

public:
	bool IsConnected();
	int Connect(string port);
	void Disconnect();

	int SetConfig(int configItem, int index, int value);
	int SetConfig(int configItem, int value);

	int SetCommand(int commandItem, int index, int value);
	int SetCommand(int commandItem, int value);
	int SetCommand(int commandItem);

	int GetConfig(int configItem, int index, int &result);
	int GetConfig(int configItem, int &result);

	int GetValue(int operatingItem, int index, int &result);
	int GetValue(int operatingItem, int &result);

	RoboteqDevice();
	~RoboteqDevice();
};

#endif

#include <string.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <jsoncpp/json/json.h>

#include "drive_api.h"

using namespace std;

static void PressEnterToExit()
{
    int c;

    cout << "Press enter to exit." << endl;;

    while( getchar() != '\n')
	{
		if (Amber::Motion::GetStopSignal())
		{
			return;
		}
	}
	Amber::Motion::SetStopSignal();
}

void WorkThread(Amber::AiosGroup *unit)
{	
	if( Amber::Motion::Replay(unit) < 0)
	{
		cout << "\033[31m" << "INFO: " << GetSystemError() << endl;
	}
}

int main(int argc, char *argv[])  
{	  
	Amber::Lookup lookup;

	std::vector < string > serial_number;
	
	Json::Reader reader;
	Json::Value root;
	 
	ifstream in("config.json", ios::binary);
	 
	if (!in.is_open())
	{
		return 0;
	}
	 
	if (reader.parse(in, root))
	{
		for (int i=0;i<root.size();i++)
		{
			serial_number.push_back(root[i]["serial_number"].asString());
		}
	}
	else
	{
		return 0;
	}

	std::shared_ptr <Amber::AiosGroup> group = lookup.GetHandlesFromSerialNumberList(serial_number);
	if (!group)
	{
		cout << "\033[31m" << "INFO: No device found on network" << endl;
		return -1;
	}

	cout << "\033[32m" << "INFO: "<< group->Size() << " devices found on network" << endl;

	auto actuator_info = group->GetActuatorInfo();

	for (auto it = actuator_info.begin(); it != actuator_info.end(); it++)
	{
		cout << "\033[34m" << "{" << endl;
		cout << "\033[34m" << "    ip = " << it->ip_ << endl;
		cout << "\033[34m" << "    serial number = " << it->serial_number_  << endl;
		cout << "\033[34m" << "    mac address = " << it->mac_address_  << endl;
		cout << "\033[34m" << "}" << endl;
  	}

	if(!group->Calibration())
	{
		cout << "\033[31m" << "INFO: " << GetSystemError() << endl;
 		return -1;
	}

	if(!group->SaveConfig())
	{
		cout << "\033[31m" << "INFO: " << GetSystemError() << endl;
 		return -1;
	}

	if(!group->Enable())
	{
		cout << "\033[31m" << "INFO: " << GetSystemError() << endl;
		return -1;
	}

	std::cout << "\033[33m"	<<"INFO: Replaying"<< endl;

	std::thread thread_running(WorkThread,group.get());

	PressEnterToExit();

	thread_running.join();
	return 0;
}



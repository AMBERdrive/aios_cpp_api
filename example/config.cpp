#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "drive_api.h"

using namespace std;

int main(int argc, char *argv[])  
{	  
	Amber::Lookup lookup;

	Json::Value root;
	Json::Value element;

	std::shared_ptr <Amber::AiosGroup> group = lookup.GetAvailableList();
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

		element["serial_number"]  = Json::Value(it->serial_number_);
		element["mac_address"]	= Json::Value(it->mac_address_);
		root.append(element);
  	}

	Json::StyledWriter writer;
	ofstream os;
	os.open("config.json");
	os << writer.write(root);
	os.close();

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

	return 0;
}





#include <iostream>

#include "drive_api.h"

int main(int argc, char *argv[])  
{	  
	Amber::Lookup lookup;

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
  	}

	return 0;
}




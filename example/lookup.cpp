#include <iostream>
#include "drive_api.h"

int main(int argc, char *argv[])  
{	  
	Amber::Lookup lookup;

	std::shared_ptr <Amber::AiosGroup> group = lookup.GetAvailableList();
	if (!group)
	{
		cout << "EVENT: No device found on network" << endl;
		return -1;
	}
	cout << "EVENT: "<< group->Size() << "devices found on network" << endl;

	auto actuator_info = group->GetActuatorInfo();

	for (auto it = actuator_info.begin(); it != actuator_info.end(); it++)
	{
		cout << "EVENT: find server ip = " << it->ip_ << endl;
		cout << "            serial number = " << it->serial_number_  << endl;
		cout << "            mac address = " << it->mac_address_  << endl;
  	}

	return 0;
}




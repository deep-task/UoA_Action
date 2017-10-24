#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <stdlib.h>

#include <sstream>
#include <iostream>
#include <string>


// Include algorithm
#include "../modules/action_generator/ActionAlgorithm.cpp"

void Callback_fromTaskExe(const std_msgs::String::ConstPtr& jsonEncodedResult_Input);
void Callback_fromSilbot(const std_msgs::String::ConstPtr& jsonEncodedResult_Input);

ros::Publisher pub_toTaskExe, pub_toSilbot;
ros::Subscriber sub_fromTaskExe, sub_fromSilbot;

int main(int argc, char **argv){

	// 1. ROS Node setting
	ros::init(argc, argv, "UoA_action_generator");
	ros::NodeHandle n;

	// 2. ROS Subscriber setting
	sub_fromTaskExe = n.subscribe("json_TaskExe", 100, Callback_fromTaskExe);
	sub_fromSilbot = n.subscribe("json_SilbotComp", 100, Callback_fromSilbot);

	// 3. ROS Publisher setting
	pub_toTaskExe = n.advertise<std_msgs::String>("json_TaskComp",100);
	pub_toSilbot = n.advertise<std_msgs::String>("json_SilbotExe",100);

	ros::spin();

	return 0;
}


void Callback_fromTaskExe(const std_msgs::String::ConstPtr& jsonEncodedResult_Input)
{

 
	std_msgs::String jsonEncodedResult_Output;

	// 1. Perform toSilbot algorithm
	ActionAlgorithm(jsonEncodedResult_Output, jsonEncodedResult_Input);

	// 2. Publish the json format string
	if (!jsonEncodedResult_Output.data.empty())
	{
		pub_toSilbot.publish(jsonEncodedResult_Output);
		std::cout << jsonEncodedResult_Output << std::endl << std::endl;
		printf("\n############################\n");
		printf("/json_SilbotExe is Transmitted to Silbot\n");
	}

}


void Callback_fromSilbot(const std_msgs::String::ConstPtr& jsonEncodedResult_Input)
{

 
	std_msgs::String jsonEncodedResult_Output;

	// 1. Perform toTask algorithm
	ActionAlgorithm(jsonEncodedResult_Output, jsonEncodedResult_Input);

	// 2. Publish the json format string
	if (!jsonEncodedResult_Output.data.empty())
	{
		pub_toTaskExe.publish(jsonEncodedResult_Output);
		std::cout << jsonEncodedResult_Output << std::endl << std::endl;
	}

}

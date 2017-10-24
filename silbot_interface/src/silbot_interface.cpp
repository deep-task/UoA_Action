#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>

// Include uoa algorithm
#include "SilbotAlgorithm.cpp"

void Callback(const std_msgs::String::ConstPtr& jsonEncodedResult_Input);

ros::Publisher pub;
ros::Subscriber sub;

int main(int argc, char **argv){

	// 1. ROS Node setting
	ros::init(argc, argv, "KIST_silbot_interface");
	ros::NodeHandle n;

	// 2. ROS Subscriber setting
	sub = n.subscribe("json_SilbotExe", 100, Callback);

	// 3. ROS Publisher setting
	pub = n.advertise<std_msgs::String>("json_SilbotComp",100);

	ros::spin();

	return 0;
}


void Callback(const std_msgs::String::ConstPtr& jsonEncodedResult_Input)
{

 
	std_msgs::String jsonEncodedResult_Output;

	// 1. Perform UoA algorithm
	SilbotAlgorithm(jsonEncodedResult_Output, jsonEncodedResult_Input);

	// 2. Publish the json format string
	if (!jsonEncodedResult_Output.data.empty())
	{
		ros::Duration(5).sleep();
		pub.publish(jsonEncodedResult_Output);
		std::cout << jsonEncodedResult_Output << std::endl << std::endl;
	}

}

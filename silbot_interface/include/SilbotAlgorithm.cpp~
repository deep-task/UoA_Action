#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "json/json.h"
#include "jsoncpp.cpp"

std::string json_str;

void SilbotAlgorithm(std_msgs::String& jsonEncodedResult_Output, const std_msgs::String::ConstPtr& jsonEncodedResult_Input){


	////////////////////////////////////////////////////////
	// Input : json Parser
	////////////////////////////////////////////////////////

	// convert to json
	json_str = jsonEncodedResult_Input->data.c_str() ; // std_msgs::String ----> std::string

	// Parser
	Json::Reader reader;
	Json::Value input_root;

	// Error check
	bool parsingRet = reader.parse(json_str, input_root);
	if (!parsingRet)
	{
		std::cout << "Failed to parse Json : " << reader.getFormattedErrorMessages();
		return;
	}



	////////////////////////////////////////////////////////
	// Example
	////////////////////////////////////////////////////////


	Json::Value input_info;
	Json::Value input_task;
	input_info = input_root["1.INFO"];
	input_task = input_root["2.Task"];


//	std::cout << input_info << std::endl;
//	std::cout << input_task << std::endl;



	
	////////////////////////////////////////////////////////
	// Output
	////////////////////////////////////////////////////////
	
	Json::Value output_root;

	// 1. Info
	Json::Value output_info;
	output_info["MODULE"] = "Kist_Silbot";
	output_info["start"] = 500;
	output_info["end"] = 900;
	output_root["1.INFO"] = output_info;

	// 2. taskCompletion
	Json::Value taskCompletion;
	taskCompletion["1.TaskNumber"] = input_task["1.TaskNumber"];
	taskCompletion["2.Behavior"] = input_task["2.Behavior"];
	taskCompletion["3.Status"] = "Completed";
	output_root["2.Task_Completion"] = taskCompletion;


	//
	Json::StyledWriter writer;
	json_str = writer.write(output_root);
//	std::cout << json_str << std::endl << std::endl;

	// convert json to String form
	jsonEncodedResult_Output.data = json_str.c_str();
	


}

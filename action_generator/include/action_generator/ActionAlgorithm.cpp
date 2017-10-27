#include <stdio.h>
#include <stdlib.h>

//#include <cstdio>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "../json/json.h"
#include "../jsoncpp.cpp"
#include "../json/json-forwards.h"

std::string json_str;

void ActionAlgorithm(std_msgs::String& jsonEncodedResult_Output, const std_msgs::String::ConstPtr& jsonEncodedResult_Input){

	/////////////////////////////////////////////////////////
	// Input : Individual task - behavior, robot_dialog (as json format)
	// Output : Task completion message
	////////////////////////////////////////////////////////




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
	input_info = input_root["INFO"];


	std::string module = input_info["MODULE"].asString();

	if (module == "TaskExecution")
	{

		Json::Value input_task = input_root["Task"];

		Json::Value output_root;
			

		// 1. Info
		Json::Value output_info;
		output_info["MODULE"] = "KIST_Action";
		output_info["start"] = 500;
		output_info["end"] = 900;
		output_root["INFO"] = output_info;
	
		// 2. taskCompletion
		output_root["Task"] = input_task;
	
		//
		Json::StyledWriter writer;
		json_str = writer.write(output_root);
	
		// convert json to String form
		jsonEncodedResult_Output.data = json_str.c_str();

	}



	if (module == "Kist_Silbot")
	{

		Json::Value input_taskcomp = input_root["Task_Completion"];

		Json::Value output_root;
			

		// 1. Info
		Json::Value output_info;
		output_info["MODULE"] = "KIST_Action";
		output_info["start"] = 500;
		output_info["end"] = 900;
		output_root["INFO"] = output_info;
	
		// 2. taskCompletion
		output_root["Task_Completion"] = input_taskcomp;
	
		//
		Json::StyledWriter writer;
		json_str = writer.write(output_root);
	
		// convert json to String form
		jsonEncodedResult_Output.data = json_str.c_str();

	}


}

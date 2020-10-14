# UoA_Action

## Overview

다양한 로봇에 범용으로 사용할 수 있는 행동 표현 생성기 개발

Authors: Ho Seok Ahn, Byeong-Kyu Ahn, JongYoon Lim <br/>
Maintainer: Byeong-Kyu Ahn, b.ahn@auckland.ac.nz <br/>
Affiliation: The University of auckland

## Installation

These packages are not support binary installation yet.


### Build from Source

        $ git clone --recursive https://github.com/deep-task/UoA_Action.git

        for 6dof
        $ git clone --recursive --branch 6dof https://github.com/deep-task/UoA_Action.git


#### Dependencies

        $ rosdep install --from-paths UoA_Action --ignore-src -r -y

        $ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r


#### Building

        $ catkin build or cakin_make


## Usage

        - Use fake renderer without real robots

        $ roslaunch uoa_bringup bringup_silbot3.launch project_path:=<your-project-path> map_name:=<map-name> use_fake_render:=true

        - Use real robots (silbot3)

        $ roslaunch uoa_bringup bringup_silbot3.launch project_path:=<your-project-path> map_name:=<map-name> use_fake_render:=false


## Nodes

UoA_Action subscribe topics /taskExecution [std_msgs/String], /perceptionResult [std_msgs/String] and publish topic /taskComplition [std_msgs/String].

Each topic has json format.

### /taskRequest

{\"header\": {\"content\": [\"robot_action\"], \"timestamp\": \"1542625360.322583913\", \"target\": [\"UOA\"], \"source\": \"UOS\"}, \"robot_action\": {\"sm\": \"neutral\", \"dialog\": \"\", \"id\": 319, \"behavior\": \"head_toss_gaze:jy\"}}

### /taskExecution

        {
            "INFO": {
                "MODULE": "UOS",
                "end": 800,
                "start": 700
            },

            "Task": {
                "TaskNumber": "1",
                "Behavior": "moving",
                "Robot_Dialog": "",
                "Human_Info": {
                    "Age": 20,
                    "Id": 1,
                    "Name": "KJH",
                    "X_Position": 0,
                    "Y_Position": 1,
                    "Z_Position": 2
                }
            }
        }


### /taskComplition

        {
            "INFO": {
                "MODULE": "UOA",
                "end": 1500,
                "start": 1300
            },
            "Task_Completion": {
                "TaskNumber": "2",
                "Behavior": "greeting",
                "Status": "Completed"
            }
        }


## Bug & Feature Requests

Please report bugs and request features using the Issue Tracker.

## Errors
- ERROR: Failed building wheel for PyAudio

sudo apt install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 ffmpeg libav-tools

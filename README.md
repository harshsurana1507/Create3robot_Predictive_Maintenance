# Data Collection and Fault Prediction for Multiple Mobile Create 3 Robots
Use https://shields.io/badges to create badges.

![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Static Badge](https://img.shields.io/badge/raspberry-purple?logo=raspberrypi)
![Passing Badge](https://img.shields.io/badge/Build-passing-green?style=for-the-badge)
![ROS Version](https://img.shields.io/badge/ROS%20version-humble-blue?style=for-the-badge)

<!-- TABLE OF CONTENTS -->
## Table of Contents
1. [About The Project](#about-the-project)
   - [Built With](#built-with)
2. [Getting Started](#getting-started)
   - [Setup](#setup)
   - [Installation](#installation)
3. [Usage](#examples-of-usages)
4. [Roadmap](#roadmap)
5. [License](#license)
6. [Contact](#contact)



<!-- ABOUT THE PROJECT -->
## About The Project

Robust predictive fault detection strategies are essential within the field of robotics. Utilizing advanced data analysis techniques, the project endeavors to anticipate errors before they manifest, thereby enabling proactive fault measures. This proactive approach not only improves thereby cultivating a more dependable and efficient robotic ecosystem.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



###  Built With
- Python 3.8-3.10
- Anaconda 2023.07-1
- MatplotLib 3.9.2
- Sklearn 1.5.2
- Pandas 2.2.3
- Numpy 2.1.1
- ROS2 Humble
- LGBM (Light Gradient Boossting Machine) 4.5.0


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

### Setup
Hardware: 
- Create3 Educational Robot
- Raspberry Pi 5 with ROS2 Humble
- PC as a server with ROS2 Humble
- Router for a Wi-Fi connection

Setup:
1. Setting up Create3 Educational Robot.
   -  Follow the steps as directed on the page: https://edu.irobot.com/create3-setup. And make sure to select "rmw_fastrtps_cpp" as the RMW_IMPLEMENTATION.
   -  If you are using multiple robot then name the robots differently. Such as I named my robot as robot01.
   -  Both the robot and the server should have same ROS version (ROS2 Humble present in the Robot at the time of project).
   -  Connect the iRobot to a Wi-Fi later the server will also be connected on the same wi-fi.
     
2. Setting up the Server for collecting data from all the topics.
   -  Follow the steps as defined on the site https://iroboteducation.github.io/create3_docs/setup/ubuntu2204/ to install ROS2 humble on the server. In step 10 select echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
   -  Connect server to the Wi-Fi (same which is used for iRobot )
   -  To check whether proper connection has been established between server and iRobot. Type command ros2 topic echo in the terminal. And you will be able to see all the topics of the robot.

### Installation

1. Download Create3_main-main zip file at the server PC and extract it.
2. Open the terminal make sure everything is updated by using
    ```sh
   sudo apt update
   sudo apt upgrade -y
   ```
3. Using terminal go to create3_main-main folder and enter the below command to install the packages on the server
    ```sh
   colcon build
   ```
4. Search for the packages create3_control, create3_msg_transfer and create3_teleop in the list provided by the command
   ```sh
   ros2 pkg list
   ```
5. If in the setup of robot you have specified a name of robot such as robot01 as I used in my case. Then change "robot01" with the name of the robot you chosed in every nodes of create3_main.
6. Run the below code to check whether robot is moving or not. With this launch file the robot will detects obstacles and move accordingly.
    ```sh
   ros2 launch create3_teleop ir_avoider.py
   ```
7. Run  the below code to check whether all the sensors are sending the data or not. With this launch file we save the data from every sensor placed on the robot on the server. Every 1 second the data is saved as an excel file according to timestamps.
    ```sh
   ros2 launch create_3_msg_transfer subscriber_launch.py
   ```
   



<!-- USAGE EXAMPLES -->
## Examples of Usages

![flowchart](https://github.com/user-attachments/assets/732162bb-8b8d-4f39-a802-67ad5ce7c578)
### Example 1 : Running the iRobot without any error: 
1. Run create_3_msg_transfer subscriber_launch.py
2. Run ros2 launch create3_teleop ir_avoider.py
3. Let the robot run for about an hour.
4. Press Ctrl+c in the terminal where launch file of ir_avoider is running.
5. Press Ctrl+c in the terminal where launch file of subscriber_launch is running.
6. Run the excel_combine.py file to combine all the data of the sensors in 1 excel sheet according to the timestamp.
7. Use the prediction.txt trained model and classification.txt trained model file. And give the location of the combined excel file.
8. In the output of prediction we will get values less than 0.5 and classification as 0.
   
### Example 2: Running the iRobot when hair or somedirt is surrounded around the wheel, which makes wheels rotation hard
1. Repeat the steps 1 till 7 as in Example 1.
2. In the output of prediction we will get values greater than 0.5 showing that maintenance is required and classification as 2, showing maintenance required of wheels.

![IMG-20240805-160439](https://github.com/user-attachments/assets/317eef01-6a48-4071-af98-eeb5fce882fc)
To get the data of the maintenance stage, I rapped the wheels with the tape.

### Example 3: Running the iRobot where IR sensor is not able to detect small height object (such as wires or cardboard/paper):
1. Repeat the steps 1 till 7 as in Example 1.
2. In the output of prediction we will get values greater than 0.5 showing that maintenance is required and classification as 1, showing maintenance required because IR sensor can't detect an object.
![IMG-20240808-135446 (1)](https://github.com/user-attachments/assets/c8cf3f88-4ce6-4157-86c6-9851cd787f36)
Video link: https://github.com/user-attachments/assets/2ca30d02-452c-4b21-b640-fb6e924bab58

Robot dragging the piece of paper, cardboard and bundle of wire without detecting it.

### Example 4: Running the iRobot when battery is less than 12%: 
1. Repeat the steps 1 till 7 as in Example 1.
2. In the output of prediction we will get values greater than 0.5 showing that maintenance is required and classification as 3, showing that battery of the robot is less, so charge it.
![Screenshot_2024-10-01-09-07-08-18_92460851df6f172a4592fca41cc2d2e6](https://github.com/user-attachments/assets/00da04b2-01b7-49dc-9d22-0d3ba613d166)
When robots battery is less than 12% the LED light on top of robot starts blinking in red color.


<!-- ROADMAP -->
## Roadmap
Description of the milestones of this project:
- [ ] ~~Connecting the Rasp. Pi 5 with the server~~
- [ ] ~~Test the send and accept function~~ 
- [ ] Searching on building up a Databank on the Server, with SQL or similar tools
- [ ] Set up a Robot for data collection
   - [ ] Install ROS2 and use the data collection node 
- [ ] Collect the data on Server
- [ ] Searching for proper analysis tools
- [ ] Try to predict the failure



## ROS2 Topics from Create3 Robot
- /battery_state : Gives information about the battery of the robot. Such as how much battery is charged, the voltage, current, temperature.
- /cliff_intensity : Gives information on how near create3 robot is to the surface using 4 sensors. 
- /dock_status : Provides 2 information firstly whether the robot can see the dock or not. And whether the robot is docked or not.
- /imu : Provide information of robots orientation, angular velocity and acceleration using imu (inertial measurement unit) sensor.
- /ir_intensity : It provides IR (infrared) intensity reading from 7 sensors. As the robot comes near to an object the value received from sensor increases.The position of 7 sensors are mentioned below.
- /ir_opcode : Opcode detected by the robot IR receivers. Used to detect the dock and virtual walls.
- /kidnap_status : Gives information about whether the robot is picked up or not.
- /mouse : Reading from mouse sensor.
- /odom : This represent an estimation of the position in free space.
- /slip_status : Provide information about whether robots wheels are slipping or not.
- /stop_status : Provide information about whether robot is stopped or moving.
- /wheel_status :  Current and PWM(pulse width modulation) readings from the robot's two wheels in addition to whether wheels are enabled.
- /wheel_ticks : Count of encoder ticks for both wheels.
- wheel_vels : Velocity of both wheels in rad/sec.


## Folder structure
Describe here, how your project is structured:
Examples:
```text
project/
```text
create3_project/
├── config/
│   ├── rviz config
│   ├── LiDAR sensor config
│   └── mapper config
├── include/
│   └── Header file of the publisher
├── launch/
│   ├── old_lidar/
│   │   └── Python files from create3_examples/create3_lidar_slam
│   ├── old_nolidar/
│   │   └── Python files based on create3_examples/create3_lidar_slam with modifications
│   ├── launch_publisher.py (C++ tester)
│   └── Python files (shortcuts) to launch all lidar or all no lidar files
├── run/
│   ├── map.py
│   ├── run_avoider.py
│   └── run_mapper.py
└── src/
    └── Implementation file of the publisher
```
Another example: A common approach for data science projects can be found [here](https://neptune.ai/blog/best-practices-for-data-science-project-workflows-and-file-organizations).

```text
create3_project/
├── data/
│   ├── external
│   ├── interim
│   ├── processed 
│   └── processes
├── models
├── src/
    ├── data
    ├── features
    └── model
```

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.


<!-- CONTACT -->
## Contact

Name:  
E-mail: 








# Machine learning enabled predictive maintenance framework for Create3 mobile robot 
![image_alt](https://github.com/harshsurana1507/Create3robot_Predictive_Maintenance/blob/92c8ae69ab1de3f565a2bcccb125c8a7de0fee8a/create3%20robot_1.jpg)
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
- LGBM (Light Gradient Boosting Machine) 4.5.0


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

### Setup
Hardware: 
- Create3 Educational Robot
- PC as a server with ROS2 Humble
- Router for a Wi-Fi connection


Setup:
1. Setting up Create3 Educational Robots.
   -  Follow the steps as directed on the page: https://edu.irobot.com/create3-setup. And make sure to select "rmw_fastrtps_cpp" as the RMW_IMPLEMENTATION.
   -  If you are using multiple robots then name the robots differently. Such as I named my robots as robot01, robot02 and so on.
   -  All the robots and the server should have same ROS version (ROS2 Humble present in the Robots at the time of project).
   -  Connect the iRobot to a Wi-Fi later the server will also be connected on the same wi-fi.
     
2. Setting up the Server for collecting data from all the topics of the robot.
   -  Follow the steps as defined on the site https://iroboteducation.github.io/create3_docs/setup/ubuntu2204/ to install ROS2 humble on the server. In step 10 select echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
   -  Connect server to the Wi-Fi (same which is used for iRobot )
   -  To check whether proper connection has been established between server and iRobot. Type command ros2 topic echo in the terminal. And you will be able to see all the topics of the robots.
The following steps can be repeated for multiple robots. Only there is a need to change the robot name in every nodes of the package. Like I have used "robot01" in all nodes.

### Installation

1. Open the terminal make sure everything is updated by using
    ```sh
   sudo apt update
   sudo apt upgrade -y
   ```
2. Using terminal go to create3_main-main folder (folder will be provided on request) and enter the below command to install the packages on the server
    ```sh
   colcon build
   ```
3. Search for the package create3_msg_transfer in the list provided by the command
   ```sh
   ros2 pkg list
   ```
4. If in the setup of robot you have specified a name of robot such as robot01 as I used in my case. Then change "robot01" with the name of the robot you chosed in every nodes of create3_main.
5. To run the robot a predeveloped code Create3 Teleoperation can be used which is on the official website of create 3, or code could be developed to run the robot autonomously by avoiding obstacles using IR sensors.
6. After adding the package from official website of Create3 as mentioned in step 6, run the below command and use the keyboard keys to check whether robot is moving or not. 
    ```sh
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
7. Run the below command to check whether all the sensors are sending the data or not. With this launch file we save the data from every sensor of the robot on the server. Data will be gathered in every 1 second, and later on will be saved as an excel file according to timestamps.
    ```sh
   ros2 launch create_3_msg_transfer subscriber_launch.py
   ```
8. Download prediction.txt and classification.txt trained model from the google drive(Link added on main page).
   



<!-- USAGE EXAMPLES -->
## Examples of Usages
![image_alt](https://github.com/harshsurana1507/Create3robot_Predictive_Maintenance/blob/f58b6a71a1200797221f1d5d65574f1631175d91/flowchart1.0.PNG)

### Example 1 : Running the iRobot without any error: 
1. Run create_3_msg_transfer subscriber_launch.py
2. Run ros2 run teleop_twist_keyboard teleop_twist_keyboard
3. Let the robot run for about an hour.
4. Press Ctrl+c in the terminal where teleop_twist_keyboard is running.
5. Press Ctrl+c in the terminal where launch file of subscriber_launch is running.
6. Run the excel_combine.py file to combine all the data of the sensors in a single excel sheet according to the timestamps.
7. Use the prediction.txt trained model and classification.txt trained model file. And give the location of the combined excel file.
8. In the output of prediction we will get values less than 0.7 .
   
### Example 2: Running the iRobot when hair or somedirt is surrounded around the wheel, which makes wheels rotation hard
1. Repeat the steps 1 till 7 as in Example 1.
2. In the output of prediction we will get values greater than 0.7 showing that maintenance is required and classification as 2, showing maintenance required of wheels.


![image_alt](https://github.com/harshsurana1507/Create3robot_Predictive_Maintenance/blob/dea14f3c8736bea9e57794c89ba15520e582e2fd/IMG-20240805-160439.jpg)

To simulate such malfunction scenario, I wrapped the wheels with the tape. So that friction is developed, which hinders smooth rotation motion of the wheels. And hence motor of the wheels has to apply high torque.

### Example 3: Running the iRobot where IR sensor is not able to detect small height object (such as wires or cardboard/paper):
1. Repeat the steps 1 till 7 as in Example 1.
2. In the output of prediction we will get values greater than 0.7 showing that maintenance is required and classification as 1, showing maintenance required because IR sensor can't detect an object.
![image_alt](https://github.com/harshsurana1507/Create3robot_Predictive_Maintenance/blob/3b9c0b02a497dc2d1fb50fc7246172cd54dab809/cliff%20error.jpg)


To gather the data for this type of scenario, the robot was run over wires, cardboard and paper without getting detected by the IR sensors. Hence robot's base is getting scratched. Moreover,the object may rub against the wheels of the robot, causing wear and tear of the wheels.

  

### Example 4: Running the iRobot when battery is less than 12%: 
1. Repeat the steps 1 till 7 as in Example 1.
2. In the output of prediction we will get values greater than 0.7 showing that maintenance is required and classification as 3, showing that battery of the robot is less, so charge it.
![image_alt](https://github.com/harshsurana1507/Create3robot_Predictive_Maintenance/blob/ed27541316f45eddea25c7d2d706493f2aabc4ff/Screenshot_2024-10-01-09-07-14-23_92460851df6f172a4592fca41cc2d2e6.jpg)
When robots battery is less than 12% the light on top of the robot starts blinking in red color.
To collect the data of this case. I ran the robot, when the battery level goes from 10% until 3%. 


<!-- ROADMAP -->
## Roadmap
Description of the milestones of this project:
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
- /tf: Provide information about transform.
- /stop_status : Provide information about whether robot is stopped or moving.
- /wheel_status :  Current and PWM(pulse width modulation) readings from the robot's two wheels in addition to whether wheels are enabled.
- /wheel_ticks : Count of encoder ticks for both wheels.
- /wheel_vels : Velocity of both wheels in rad/sec.



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.


<!-- CONTACT -->
## Contact

Name: Harsh Vardhan Surana  
E-mail: harshsurana1507@gmail.com








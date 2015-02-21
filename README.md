# ROS support for 1:8 scale vehicle

Hardware
---------------------
*  Motorcontroller:
 * http://vedder.se/2014/01/a-custom-bldc-motor-controller/
* IMU
 * Razor IMU
 * STM32F3 Discovery board
* US Sensors
 * SRF08
 * USB 2 I2C
  * http://www.robot-electronics.co.uk/htm/usb_i2c_tech.htm

We have forked Benjamins code for the motor controller.
Our fork is on github at https://github.com/finkultur/bldc .


1. Installation
---------------------
  1.1 Install ROS by following the instruction at http://wiki.ros.org/indigo/Installation/Ubuntu  

1.2 Clone the dat295 repository: `git clone https://<username>@bitbucket.org/thpe/dat295.git`  
1.3 Fix udev rules  
  1.3.1 `cp dat295/99-usb-serial.rules /etc/udev/rules.d/`  
  1.3.2 Reload rules: `udevadm control --reload-rules`  

1.4 Build and flash software for the motor controller board  
  1.4.1 Clone the bldc repository: `git clone https://github.com/finkultur/bldc.git`  
  1.4.2 `cd bldc`  
  1.4.3 Compile and build: `make`  
  1.4.4 Flash motor controller: `st-flash write build/BLDC_4_ChibiOS.bin 0x8000000`  

1.5 Build and flash software for the IMU  
  1.5.1 `cd dat295/vehicle/DiscoveryBoard`  
  1.5.2 Compile and build: `make`  
  1.5.3 Flash: `st-flash write build/stm32f3discovery-demo.bin 0x8000000`  

1.6 Build all ROS modules
  1.6.1 `cd dat295/vehicle/catkin_ws/`  
  1.6.2 `catkin_make`  
  1.6.3 `source devel/setup.bash`  

1.7 Install joystick drivers:
  1.7.1 Install xboxdrv through your regular packet manager, e.g: `apt-get install xboxdrv`  
  1.7.2 Run `lsusb` to find out productId and vendorId of joystick  


2. Drive manually with Joystick
---------------------
2.1 On ROS-master (the car), run: `roslaunch /dat295/vehicle/catkin_ws/src/master_dbj.launch`  
2.2 On ROS-slave, run (in separate terminals):  
  2.2.1  `xboxdrv --device-by-id <vendorId>:<productId> --type xbox360`  
  2.2.2  `roslaunch dat295/vehicle/catkin_ws/src/slave.launch`  


3. Drive by Autopilot
---------------------
3.1  Create a set of waypoints with tracker.py: `dat295/vehicle/tracker/tracker.py`  
3.2 Output waypoints (press SPACE in tracker) and copy these into top of `dat295/vehicle/catkin_ws/src/autopilot/scripts/autopilot.py`  
3.3 Launch the autopilot by running `roslaunch dat295/vehicle/catkin_ws/src/master_autopilot.launch`  


4. Tracker
---------------------
4.1 To track position of car, run: `dat295/vehicle/tracker/tracker.py --track <track>.track --plot`  
4.2 When editing track, see top of script for instructions  


5. Configuration
---------------------
Configuration can be done by editing the files  
  dat295/vehicle/catkin_ws/src/autopilot/scripts/config.py  
  dat295/vehicle/catkin_ws/src/master_autopilot.launch  
  dat295/vehicle/catkin_ws/src/master_dbj.launch  
  dat295/vehicle/catkin_ws/src/slave.launch  


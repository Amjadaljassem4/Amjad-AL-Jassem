
# LEGO Robot

![The robot](http://robotsquare.com/wp-content/uploads/2013/10/45544_crane-550x227.jpg)


## Introduction

Welcome to the LEGO® MINDSTORMS® EV3 robot repository! Here, you'll find everything you need to create your very own sorting robot using a Lego EV3 Mindstorms Kit and Pybricks software. 
This robot is truly remarkable, boasting a variety of impressive capabilities:
Object Manipulation: With precision and ease, the robot can pick up and place objects exactly where you want them.
Color Detection: Utilizing advanced technology, the robot can identify and determine the colors of objects it encounters.
Sorting Functionality: Impressively, the robot can sort objects based on their colors, directing them to different locations as per your instructions.
Collaborative Potential: Through seamless communication with other robots, this creation can expand the possibilities of where objects can be placed.
Contained within this readme is a comprehensive guide that will walk you through the installation process and demonstrate how to use the program effectively. 

Additionally, you'll find detailed information on the various features and functionalities of this extraordinary robot. Get ready to embark on an exciting journey into the world of robotics!


## Getting started

To get started, you will need to install Python and VS Code on your computer. You can download the latest version of Python from the [official website](https://www.python.org/downloads/). Once you have downloaded and installed Python, you can download and install [Visual Studio Code](https://code.visualstudio.com/download).

Next, you will need to install the Pybricks library. Pybricks is a Python library that provides an easy-to-use interface for programming LEGO robots. You can install Pybricks using pip, which is a package installer for Python, or follow [these instructions](https://github.com/pybricks/pybricksdev). To install with pip, open the terminal or command prompt and run the following command:
```
pip install pybricksdev
```
This will install the latest version of Pybricks on your computer.

Next we recommend installing the LEGO® MINDSTORMS® EV3 MicroPython VS Code extension to help with connecting to the robot. This can be done by searching for '***lego-education.ev3-micropython***' in the extenstions tab in VS Code and downloading the showing extension.

Now comes the part where you download the code that the robot will run. In [releases](https://github.com/Ozzcarr/lego-robot/releases), from the latest release, download the **lego-robot.zip** and extract the folder inside to a place of your liking. In VS Code, open the explorer by using the button on the left or by pressing Ctrl+Shift+E, click on "Open Folder" and open the folder you just extracted from the zip file.


## Building and running

In this part you will get to know how to get the robot up and running. This will include:
- Connecting the robot to the computer
- Uploading the code and running the program
- Calibrating one robot
- Running the robot 

### Connect to PC

To connect the robot to the PC, ensure that the robot is powered on and connected to the PC with a USB cable. Then the steps are the following:
1. In VS Code, open the explorer by using the button on the left or by pressing Ctrl+Shift+E.
2. Click on "EV3DEV DEVICE BROWSER" in the bottom of the explorer to extend it.
3. Click on "Click here to connect a device".
4. You should now see your robot in the menu that popped up. Click on it to connect.

### Run the program

To run the program you first need to upload the code to the robot. After you have connected the robot to the PC, this can be done by clicking the download button in the "EV3DEV DEVICE BROWSER". To run the program and run it at the same time, press F5.

To run the code once it is uploaded, follow these steps:
1. Go to file browser.
2. Click on the folder "lego-robot".
3. Click on "main.py".

The program should now start running.

### Calibrating one robot

When the program is running, press left-button to enter default mode, where only a single robot is used. The robot will then calibrate, it is important that the robot can move freely at this phase. At the end of the calibration phase the robot will play a tune.

After the calibration it is time to set locations. The robot will ask for a pickup location. Use the buttons (up, down, left, right) to navigate to the pickup location, then press the middle button in order to save the location.

The next step is to set drop-off locations. Place an item at the pickup location and press the middle button on the robot. The robot will pickup the item and read the color of it. Then use the buttons to navigate to the drop-off location of that color. When at the drop-off location, press the middle button to save it. Then place a new item at the pickup location and press the middle button, the process is then repeated. When all items has been calibrated, press the middle button, if there is no item at the pickup location, the calibration of colors will end.

### Connecting to another robot

To connect two robots with each other the following steps need to be made.

On both robots the bluetooth visibility needs to be on. This can be achieved via the following steps.

1. In the menu of the robot open "Wireless and Networks".
2. Select Bluetooth.
3. Turn on Bluetooth.
4. Turn "Visible" on.

After visibility has been turned on, on both robots, the robots need to be paired. Once they are paired, do not click connect in the menu that appears. The connection will be made when you run your programs, as described below.

1. In the bluetooth settings press "Start Scan" on both robots.
2. Wait for the search to find both robots and select the device you want to connect to.
3. Press "Pair" on the following screen after making sure it is the correct device, the same code should appear on both devices.
4. Confirm the pass key and press accept.

The robots are now connected.

### Running

#### One robot
After the setup phase the robot will enter the main loop. The program will repeat the following:
* Try to pickup an item at the pickup location.
* If there is an item at the pickup location, the robot will read the color of the item and determine the location that matches the color best. Then the robot will move to that location and drop off the item there. 
* If there is no item at the pickup location, the robot will wait for few seconds and then look for an item again.


## Features

US01: As a customer, I want the robot to pick up items.
 US02: As a customer, I want the robot to drop off items.
 US03: As a customer, I want the robot to be able to determine if an item is present at a
given location based on size (large, medium, and small)
 US04: As a customer, I want the robot to tell me the colour of an item.
 US05: As a customer, I want the robot to drop items off at different locations based on
the colour of the item.
 US06:As a customer, I want the robot to be able to pick up items from elevated positions.
 US07: As a customer, I want to be able to calibrate maximum of three different colors
and assign them to specific drop-off zones.
Updated requirements:
 US01B: As a customer, I want the robot to pick up items from a designated position
 US02B As a customer, I want the robot to drop items off at a designated position
 US04B: As a customer, I want the robot to tell me the color of an item at a designated
position.
Extra Requirements:
 US08: As a customer, I want the robot to check the pickup location periodically to see if
a new item has arrived.
 US09: As a customer, I want the robots to sort items at a specific time.
 US10: As a customer, I want two robots (from two teams) to communicate and work
together on items sorting without colliding with each other.
 US11: As a customer, I want to be able to manually set the locations and heights of one
pick-up zone and two drop-off zones. (Implemented either by manually dragging the arm
to a position or using buttons).


# StrykeForce NRG
## How to use this GitHub repository
This site is home to all the code necessary to get your Arduino Robot Car
up and running. Here we provide instructions on how to download the Arduino
code editor, how to download and install the code that runs the car,
and how to upload the code to your Arduino.

You will also find examples of how to have your Arduino Robot Car drive itself
using the built-in actions developed by Kyle Batzer. Descriptions of each
built-in action are also provided in this below. Some of these actions can be
tweaked by changing some numeric variables in the code. The names of these
variables, their location, and what they do are also documented below. We will
also be including a section on how to write your own actions and integrate them
into the existing code.

We also provide links to helpful resources to get started learning how to
program the Arduino at your own pace.

## Downloading Arduino IDE and the Robot Code

### Download and Install Arduino IDE
Arduino IDE is the tool we will use to edit the code running on the Arduino.
1. Go to https://arduino.cc/en/Main/Software
2. If using Windows, select "Windows Installer, for Windows XP and up"
3. Select "Just Download"
4. Select run.
5. Go through the prompts shown by the installer to finish installing Arduino
IDE

### Download and Install the code
1. Download the code from this page by selecting the green "Clone or download"
button.
2. Select the "Download ZIP" button.
3. Extract the ZIP file.
4. Open Arduino IDE.
5. At the top left of Arduino IDE, Select "File", then "Open...".
6. Navigate to the extracted folder "StrykeForce-NRG-master", then into the
 "NRG_TankControl" folder.
7. Select NRG_TankControl.ino
8. Select Open.

You should now have the tank control code open in Arduino IDE.

**Note: Using "Save as..." to save another copy of this file is not
recommended.**
This code relies on code in other files in the same directory in order to
function. Without these files, you will not be able to compile or upload your
code.

### Upload the code to your Arduino Robot Car
1. Open NRG_TankControl.ino in Arduino IDE if you have not already done so.
2. In Arduino IDE, select the green right-arrow button in the upper left corner
of the window. This is the Upload button.
3. Check the black message window below for errors. If everything went well,
a message will be displayed above the black box reading "Upload complete".

## Using the built-in robot actions
After downloading the code, you'll find the place to store your Arduino Robot
Car action sequences on line 39 of NRG_TankControl.ino. On this line, a
variable called `driveStateSequence`. This is a list of comma-separated
commands the robot will follow, starting with the first command and stopping
after completing the last command. Initially, the line will look like this:

```C+
DriveState_E driveStateSequence[]     = {CONTROLLER};
```

Here, the only command is `CONTROLLER`, which tells the car to listen to
controller commands while the car is on.

You can chain multiple commands in a list to have the Arduino Robot Car
perform more complicated actions on its own. For example, here's a command
sequence where the Arduino Robot Car drives itself autonomously before giving
control back to the BlueTooth controller:

```C+
DriveState_E driveStateSequence[] = {TRACK_LINE_ULTRASONIC, // 1.
                                     LEFT_90,               // 2.
                                     FORWARD_ULTRASONIC,    // 3.
                                     LEFT_90,               // 4.
                                     TRACK_LINE_ULTRASONIC, // 5.
                                     RIGHT_45,              // 6.
                                     FWD_6IN,               // 7.
                                     FWD_6IN,               // 8.
                                     FWD_6IN,               // 9.
                                     RIGHT_45,              // 10.
                                     FWD_TILL_LINE,         // 11.
                                     LINE_TRACK_RESYNC,     // 12.
                                     CONTROLLER};           // 13.
```

This sequence tells the Arduino Robot Car several things to do:
1. Use the line tracker sensor to follow a line until the ultrasonic sensor
sees a wall.
2. Turn left 90 degrees.
3. Drive forward until the ultrasonic sensor sees a wall.
4. Turn left 90 degrees.
5. Use the line tracker sensor to follow a line until the ultrasonic sensor
sees a wall.
6. Turn right 45 degrees.
7. Drive forward 6 inches.
8. Drive forward 6 inches.
9. Drive forward 6 inches.
10. Turn right 45 degrees.
11. Drive Forward until the line tracker sensor detects a line.
12. Follow the line until the line cannot be detected.
13. Give control to the Bluetooth Controller

As the Arduino Robot Car moves from one action to the next, we say that it is
*changing states* or moving to the next *drive state* in the *drive state
sequence*. When one *drive state* is able to move onto the next action in the
list, that means it must have an *exit condition*. Another name for an
*exit condition* is *terminating condition*
which tells the car to stop the current action and go to the next one. You can
see which *drive states* have *exit conditions* in the next section.

## Descriptions of built-in robot actions

NRG_TankControl.ino has several built-in actions programmed and ready to use.
Some of the actions can be tweaked by modifying global variables which appear
in the descriptions (and in the code) with NAMES_LIKE_THIS.
Here is a list of action names and what they do.

| Action Name | Description |
| ----------- | ----------- |
| CONTROLLER |  Controller inputs are used to drive car. |
| TRACK_LINE | Basic line tracking using the line tracking sensor. As of 09/18/2019, this action will never advance to the next action in the list, so only use this as the last thing you want your car to do.
| TRACK_LINE_ULTRASONIC |  Line tracking with exit condition based on ultrasonic sensor. |
| LINE_TRACK_RESYNC | Line tracking with exit condition based on being aligned for LINE_TRACKED_RESET iterations |
| FORWARD_ULTRASONIC | Move forward until ultrasonic sensor detects object within specified distance |
| FWD_TILL_LINE | Move forward until any line tracker module detects a line |
| DELAY_3S | Stop car for 3 seconds |
| LEFT_90 | Left 90 degree turn based on time. Note that the time required will vary with battery charge level. |
| RIGHT_90 | Right 90 degree turn based on time. Note that the time required will vary with battery charge level. |
| RIGHT_45 | Right 45 degree turn based on time. Note that the time required will vary with battery charge level. |
| FWD_6IN | Move forward 6 inches based on time. Note that the time required will vary with battery charge level. |
| BACK_6IN | Move backward 6 inches based on time. Note that the time required will vary with battery charge level. |
| AUTO | Move forward, sweeping ultrasonic sensor. If an object is detected, turn in the opposite direction |
| AUTO_RIGHT | Turn right for 500 ms and return to AUTO |
| AUTO_LEFT | Turn left for 500 ms and return to AUTO |


Here are the global variables defined near the top of NRG_TankControl.ino
and how they work

| Variable Name | Description | Default Value | Valid Values |
| ------------- | ----------- | ------------- | ------------ |
| LEFT_SCALE | Modifies how quickly the left motors move. Helps remove natural "drift" between the left and right motors | 1.0 | Any positive number |
| RIGHT_SCALE |  Modifies how quickly the RIGHT motors move. Helps remove natural "drift" between the left and right motors | 1.0 | Any positive number |
| LINE_TRACKED_RESET | Determines how many times LINE_TRACK_RESYNC state will check for a line once the car loses track of the line its on. | 100 | Any positive integer |
| SPEED | Percentage controlling how quickly the motors will move. It is not recommended to change this value | 50 | Integers from 0 to 100 |

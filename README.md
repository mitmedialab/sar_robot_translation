# sar\_robot\_translation 

SAR Robot Translation is a ROS node that translates generic robot commands to
platform-specific robot commands.

## Configure and run

From the "sar\_robot\_translation/src" directory, execute the
robot\_translation\_node.py file:

`./robot_translation_node.py`

or 

`python robot_translation_node.py`

On startup, this program will read the configuration file located in the
"sar\_robot\_translation/src/" directory. The file is written in JSON. The
program expects the configuration file to be called
"robot\_translation\_config.json". The configuration includes the following
options:

- which\_robot: This ROS node will translate the generic robot commands
  received into platform-specific robot commands for the robot indicated here.
  Available robot types include: JIBO, SPRITE, SIMULATED.

If roscore is not running, the program will print a message saying that it is
unable to register with the master node, and will keep trying to connect.

## ROS messages

### SAR Robot Command messages

The program subscribes to
"/[sar\_robot\_command\_msgs](https://github.com/personal-robots/sar_robot_command_msgs
"sar_robot_command_msgs")/RobotCommand" on the ROS topic "/robot\_command".

The program subscribes to
"/[sar\_robot\_command\_msgs](https://github.com/personal-robots/sar_robot_command_msgs
"sar_robot_command_msgs")/RobotState" on the ROS topic "/robot\_state".

See
[sar\_robot\_command\_msgs](https://github.com/personal-robots/sar_robot_command_msgs
"sar_robot_command_msgs") for more info about these messages.

### Robot-specific messages

The program will publish messages to robot-specific topics depending on which
robot is set in the configuration file. 

TODO: Determine message types and ROS topic names for each robot platform.

## Version and dependency notes

This program was built and tested with:

- Python 2.7.6 
- ROS Indigo
- sar\_robot\_command\_msgs 1.0.0
- Ubuntu 14.04 LTS (64-bit)

## Bugs and issues

Please report all bugs and issues on the [sar\_robot\_translation github issues
page](https://github.com/personal-robots/sar_robot_translation/issues).

## TODO

- Fill out functions to translate generic robot commands to platform-specific
  commands
- Check that robot command message data received is in a valid format before
  passing to platform-specific translators
- Fill out function for doing something useful with robot state information

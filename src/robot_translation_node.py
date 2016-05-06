#!/usr/bin/env python

# Jacqueline Kory Westlund
# May 2016
#
# The MIT License (MIT)
#
# Copyright (c) 2016 Personal Robots Group
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy # ROS
import json # to read in JSON config file
from sar_robot_command_msgs.msg import RobotCommand # ROS msgs

# The SAR robot translation node subscribes to the robot_command topic and 
# translates any robot commands received from the generic format to platform-
# specific commands that are sent to the specific robot being used (specified
# in the config file).
class robot_translation():
    """ Robot translation node """

    def __init__(self):
        """ Initialize anything that needs initialization """
        # parse config file to find out which robot to send to
        try:
            with open ("robot_translation_config.json") as json_file:
                json_data = json.load(json_file)
            print(json_data)
            self.which_robot = json_data['which-robot']
        except ValueError as e:
            print('Error! Could not open or parse json config file!'
                + '\n  Did you use valid json?\nError: %s' % e)
        except IOError as e:
            print('Error! Could not open or could not parse json '
                   +'config file!\n  Does the file exist in this directory?'
                   + '\nError: %s' % e)


    def run_robot_translation_node(self):
        """ wait for robot commands and pass them on """
        # start ros node
        rospy.init_node('robot_translation_node', anonymous=True)
        rospy.loginfo("Robot translation node starting up!")

        # subscribe to /robot_command topic
        rospy.Subscriber('robot_command', RobotCommand, robot_command_callback)

        # keep python from exiting until this node is stopped
        rospy.spin()


    def robot_command_callback(data):
        """ translate the robot command for the specific platform! """
        print(data)

        # TODO check that data is valid (once we finish defining command format!)

        # pass command in platform-specific way
        # send to jibo...
        if (self.which-robot == 'JIBO'):
            self.send_to_jibo(data)
        # send to spritebot...
        elif (self.which-robot == 'SPRITEBOT'):
            self.send_to_spritebot(data)
        # send to other robot...
        elif (self.which-robot == 'OTHER_ROBOT'):
            self.send_to_other_robot(data)


    def send_to_spritebot(data):
        """ translate robot command to format spritebot uses """
        # TODO send command to spritebot
        print("TODO send to spritebot!")

    
    def send_to_jibo(data):
        """ translate robot command to format jibo uses """
        # TODO send command to jibo
        print("TODO send to jibo!")
    

    def send_to_other_robot(data):
        """ translate robot command to format other robot uses """
        # TODO send command to other robot
        print("TODO send to other robot!")


if __name__ == '__main__':
    # run the node!
    try:
        node = robot_translation()
        node.run_robot_translation_node()

    # if roscore isn't running or shuts down unexpectedly
    except rospy.ROSInterruptException: 
        print ('ROS node shutdown')
        pass


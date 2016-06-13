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
from sar_robot_command_msgs.msg import RobotState # ROS msgs

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

        # subscribe to /robot_command topic to get command messages
        # for the robot
        rospy.Subscriber('robot_command', RobotCommand,
                self.on_robot_command_msg)

        # subscribe to /robot_state topic to get status messages from
        # the robot
        rospy.Subscriber('robot_state', RobotState, self.on_robot_state_msg)

        # publish to robot-specific topic to pass commands to robots
        # if robot is jibo...
        if (self.which-robot == 'JIBO'):
            pass
            # TODO what topic and what message type for Jibo?
            #self.jibo_pub = rospy.Publisher('jibo_command', JiboMessage,
                    #queue_size = 10)
        # if robot is a SPRITE robot...
        elif (self.which-robot == 'SPRITEBOT'):
            pass
            # TODO what topic and what message type for SPRITE robots?
            #self.sprite_pub = rospy.Publisher('sprite_command', SpriteMessage,
                    #queue_size = 10)
        # fill in for any other robots
        #elif (self.which-robot == 'OTHER_ROBOT'):
            # TODO what topic and what message type for other robot?
            #self.other_pub = rospy.Publisher('other_command', OtherMessage,
                    #queue_size = 10)

        # keep python from exiting until this node is stopped
        rospy.spin()


    def on_robot_state_msg(data):
        """ receive status messages from robots """
        print(data)
        # TODO do something with robot status messages?


    def on_robot_command_msg(data):
        """ translate the robot command for the specific platform! """
        print(data)

        # TODO check that data is valid after we finalize command format!

        # pass command in platform-specific way
        # send to jibo...
        if (self.which-robot == 'JIBO'):
            self.send_to_jibo(data)
        # send to spritebot...
        elif (self.which-robot == 'SPRITEBOT'):
            self.send_to_spritebot(data)
        # fill in for any other robots:
        #elif (self.which-robot == 'OTHER_ROBOT'):
            #self.send_to_other_robot(data)


    def send_to_sprite(data):
        """ translate robot command to format SPRITE robot uses """
        # TODO send command to SPRITE robot
        print("TODO send to sprite robot!")

        # TODO create ROS message that will be sent to the SPRITE
        # robot
        #msg = CoRDialMessage() # TODO what kind of rosmsg?

        # TODO does it need a header? If so add imports
        # add header
        #msg.header = Header()
        #msg.header.stamp = rospy.Time.now()

        # SPRITE robots use the CoRDial system to manage speech and
        # behavior. CoRDial requires each message have a unique ID
        # string, so we generate one. Any time we see the same string
        # message, we want to generate the same ID, so that IDs are
        # uniquely paired with messages. CoRDial uses this ID to
        # cache robot speech and behavior. Ideally, CoRDial would be
        # generating and tracking its own unique keys when it caches
        # messages, but since it doesn't, the current workaround is
        # to generate and send what we hope are unique IDs appended
        # to the beginning of each message string.
        # 
        # Note that the built-in python hash function may not produce
        # identical results across systems -- e.g., it may produce
        # different hashes on 32-bit versus 64-bit systems. It will
        # suffice here because we don't care about other people being
        # able to generate the same hashes -- this node just needs to
        # be able to generate the same hash each time.
        message = "[" + hash(data.properties) + "] " + properties

        # TODO add message string with the ID to the CoRDial message

        # TODO send message - what topic?
        #self.cordial_topic.publish(msg)
        #rospy.loginfo(msg)

    
    def send_to_jibo(data):
        """ translate robot command to format jibo uses """
        # TODO send command to jibo
        print("TODO send to jibo!")

        # TODO create message to send
        # TODO fill message with data from RobotCommand
        # TODO send message
    

    def send_to_other_robot(data):
        """ translate robot command to format other robot uses """
        # fill in to send commands to any other robot
        print("TODO send to other robot!")



if __name__ == '__main__':
    # run the node!
        node = robot_translation()
        node.run_robot_translation_node()

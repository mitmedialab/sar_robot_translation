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
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header # standard ROS Header
import re # regular expression
from std_msgs.msg import String
import os, sys
from sar_jibo_command_msgs.msg import JiboSpeech, JiboAnimation, JiboLookat
import Queue

class robot_translation():
    """ The SAR robot translation node subscribes to the robot_command topic and
    translates any robot commands received from the generic format to platform-
    specific commands that are sent to the specific robot being used (specified
    in the config file).
    """

    def __init__(self):
        """ Initialize anything that needs initialization """
        try:
            # If we are not running the robot_translation node from the node's
            # src/ directory, we may need to get the full path to that directory
            # so that we can find the config file (a relative path to the config
            # file wouldn't work.)
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                os.path.dirname(sys.argv[0])))
            config_file = os.path.join(__location__,
                    'robot_translation_config.json')
            rospy.loginfo('robot translation config = ' + config_file)

            # Parse config file to find out which robot to send to.
            with open (config_file) as json_file:
                json_data = json.load(json_file)
            rospy.loginfo("Got config:\n" + str(json_data))
            if ("which_robot" in json_data):
                self.which_robot = json_data["which_robot"]
                rospy.loginfo("Found which robot: " + self.which_robot)
            else:
                rospy.loginfo("Could not read which robot to use! Expected "
                    + "option \"which_robot\" to be in the config file. "
                    + "Defaulting to SIMULATED robot.")
                self.which_robot = "SIMULATED"
        except ValueError as e:
            rospy.logerr("Error! Could not open or parse json config file!"
                + "\n  Did you use valid json?\nError: %s" % e)
        except IOError as e:
            rospy.logerr("Error! Could not open or could not parse json config "
               + "file!\n  Does the file exist in this directory?"
               + "\nError: %s" % e)


    def run_robot_translation_node(self):
        """ Wait for robot commands and pass them on. """
        # Start ros node.
        rospy.init_node('robot_translation_node', anonymous=True)
        rospy.loginfo("Robot translation node starting up! Configured to send "
               "to " + self.which_robot + " robot.")

        # Obtain participant names.
        self.child_name = rospy.get_param('/sar/global/_child_name')
        self.guardian_name = rospy.get_param('/sar/global/_guardian_name')

        # Subscribe to /sar/robot_command topic to get command messages for the
        # robot.
        rospy.Subscriber('/sar/robot_command', RobotCommand,
                self.on_robot_command_msg)
        rospy.loginfo("Subscribed to 'robot_command' topic.")

        # Subscribe to /sar/robot_state topic to get status messages from the
        # robot.
        rospy.Subscriber('/sar/robot_state', RobotState, self.on_robot_state_msg)
        rospy.loginfo("Subscribed to 'robot_state' topic.")

        # Publish to robot-specific topic to pass commands to robots.
        # If robot is Jibo...
        if (self.which_robot == 'JIBO'):
            rospy.loginfo('setting up jibo communication')

            # Subscribe to jibo state messages.
            rospy.Subscriber('/sar/jibo/state', RobotState, self.on_jibo_state)
            rospy.loginfo("Subscribed to '/sar/jibo/state' topic.")

            # Subscribe to user head position messages (so we can do head
            # tracking with Jibo).
            rospy.Subscriber('/sar/perception/user_head_position_rf', Vector3,
                self.user_head_position_jibo_rf_callback)

            # We will publish lookat, speech, and animation command messages to
            # Jibo, and we will publish robot state messages as Jibo.
            self.jibo_lookat_pub = rospy.Publisher('/sar/jibo/lookat',
                JiboLookat, queue_size=10)
            self.jibo_speech_pub = rospy.Publisher('/sar/jibo/speech',
                JiboSpeech, queue_size=10)
            self.jibo_animation_pub = rospy.Publisher('/sar/jibo/animation',
                JiboAnimation, queue_size=10)
            self.robot_state_pub = rospy.Publisher('/sar/robot_state',
                RobotState, queue_size=10)

            # Initialize other Jibo-specific variables.
            self._is_jibo_ready = True
            self._is_robot_ready = True
            # Set the default position of the user head.
            self._current_user_head_pos_jibo_rf = Vector3()
            self._current_user_head_pos_jibo_rf.x = 1
            self._current_user_head_pos_jibo_rf.y = -0.4
            self._current_user_head_pos_jibo_rf.z = 0.8

        # If robot is a SPRITE robot...
        elif (self.which_robot == 'SPRITE'):
            pass
            # TODO what topic and what message type for SPRITE robots?
            #self.sprite_pub = rospy.Publisher('sprite_command', SpriteMessage,
                    #queue_size = 10)
            #rospy.loginfo("Will publish to 'sprite_command' topic.")

        # If robot is a simulated robot...
        elif (self.which_robot == 'SIMULATED'):
            self.robot_sim_pub = rospy.Publisher('robot_sim_command',
                    RobotCommand, queue_size = 10)
            rospy.loginfo("Will publish to 'robot_sim_command' topic.")

        # Fill in for any other robots!
        #elif (self.which_robot == 'OTHER_ROBOT'):
            # TODO what topic and what message type for other robot?
            #self.other_pub = rospy.Publisher('other_command', OtherMessage,
                    #queue_size = 10)

        # Keep python from exiting until this node is stopped.
        rospy.spin()


    def on_robot_state_msg(self, data):
        """ Receive status messages from robots. """
        #rospy.loginfo("Got message:\n" + str(data))
        # TODO do something with robot status messages?
        pass


    def on_robot_command_msg(self, data):
        """ Translate the robot command for the specific platform! """
        rospy.loginfo("Got message:\n" + str(data))
        # TODO check that data is valid before trying to parse!

        # Pass command in platform-specific way.
        # Send to Jibo...
        if (self.which_robot == 'JIBO'):
            # Send a \fake\ conceptual robot state signaling that the robot is
            # busy.
            self._is_robot_ready = False
            conceptual_robot_state = RobotState()
            conceptual_robot_state.header = Header()
            conceptual_robot_state.header.stamp = rospy.Time.now()
            conceptual_robot_state.doing_action = True
            conceptual_robot_state.is_playing_sound = True
            self.robot_state_pub.publish(conceptual_robot_state)
            # Parse and send the command to Jibo.
            self.send_to_jibo(data)

        # Send to SPRITE robot...
        elif (self.which_robot == 'SPRITE'):
            self.send_to_sprite(data)

        # Send to simulated robot...
        elif (self.which_robot == 'SIMULATED'):
            self.send_to_simulated(data)

        # Fill in for any other robots:
        #elif (self.which_robot == 'OTHER_ROBOT'):
            #self.send_to_other_robot(data)


    def send_to_sprite(self, data):
        """ Translate robot command to format SPRITE robot uses. """
        # TODO send command to SPRITE robot
        rospy.logwarn("TODO send to sprite robot!")

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
        # Some people may provide a unique ID as part of an incoming
        # RobotCommand message, so if one is provided, we use that
        # instead of generating our own.
        #
        # Note that the built-in python hash function may not produce
        # identical results across systems -- e.g., it may produce
        # different hashes on 32-bit versus 64-bit systems. It will
        # suffice here because we don't care about other people being
        # able to generate the same hashes -- this node just needs to
        # be able to generate the same hash each time.
        if not data.id:
            message = "[" + str(hash(data.properties)) + "] " + properties
        else:
            message = "[" + data.id + "] " + properties

        # TODO add message string with the ID to the CoRDial message

        # TODO send message - what topic?
        #self.cordial_topic.publish(msg)
        #rospy.loginfo(msg)


    def on_jibo_state(self, data):
        """ Receive Jibo status messages. """
        self._is_jibo_ready = not (data.is_playing_sound or data.doing_action)


    def command_to_jibo_behavior_queue(self, properties):
        """ Translate a RobotCommand string to commands to send to Jibo. """
        jibo_behavior_queue = Queue.Queue()
        speech_parameters = {}

        # Replace child-name and guardian-name with actual values.
        properties = properties.replace("[child-name]", self.child_name)
        properties = properties.replace("[guardian-name]", self.guardian_name)

        # Extract speech parameters.
        if "(" in properties:
            regex = re.compile("(\([\w\s,.:]*?\))")
            p = regex.match(properties)
            sp = None
            if p != None:
                sp = p.group()
            if sp != None:
                _temp = properties.split(sp)
                properties = _temp[1].lstrip()
                # Parse sp to parameters.
                sp = sp.replace('(',"")
                sp = sp.replace(')',"")
                sp_elements = sp.split(',')
                for elem in sp_elements:
                    elem = elem.strip()
                    elem = elem.lstrip()
                    _elem = elem.split(':')
                    _key = _elem[0]
                    _value = float(_elem[1])
                    speech_parameters[_key] = _value

        # Break the RobotCommand string into Jibo unit commands for speech and
        # animations. Actions/animations are enclosed in angle brackets; all
        # else is considered speech.
        while "<" in properties:
            regex = re.compile("(<[a-z\-_]*?\s?,?\s?[b|nb]*>)")
            _first_match = None
            for matches in regex.finditer(properties):
                _first_match = matches.group()
                break
            _seg = properties.split(_first_match)
            _speech = _seg[0]
            _speech = _speech.strip()
            _speech = _speech.lstrip()
            if _speech != "":
                jibo_behavior_queue.put(_speech)
            jibo_behavior_queue.put(_first_match)
            properties = _seg[1]
        # If there are no actions/animations, everything is speech.
        if properties != "":
            jibo_behavior_queue.put(properties)

        rospy.loginfo("--------------------------------")
        for elem in list(jibo_behavior_queue.queue):
            rospy.loginfo(elem)
        rospy.loginfo("--------------------------------")

        return jibo_behavior_queue, speech_parameters


    def send_to_jibo(self, data):
        """ Translate robot command to format Jibo uses. """
        # TODO: should spin out a thread for this...
        if data.command == RobotCommand.SLEEP:
            #msg.signal = JiboCommand.SLEEP
            #self.jibo_command_pub.publish(msg)
            # TODO: Deal with SLEEP commands.
            pass

        elif data.command == RobotCommand.WAKEUP:
            #msg.signal = JiboCommand.WAKEUP
            #self.jibo_command_pub.publish(msg)
            # TODO: Deal with WAKEUP commands.
            pass

        elif data.command == RobotCommand.DO:
            #msg.signal = JiboCommand.DO
            # prepare sub-command queue
            behavior_queue, speech_parameters = \
                self.command_to_jibo_behavior_queue(data.properties)

            # Send Jibo speech and action commands while we have some to send.
            while not behavior_queue.empty():
                if self._is_jibo_ready:
                    _content = behavior_queue.get()
                    if _content == None:
                        continue
                    rospy.loginfo('_content = ' + _content)

                    # Send animation commands if we have them.
                    if "<" in _content:
                        _content = _content.replace("<", "")
                        _content = _content.replace(">", "")
                        _anim = _content.split(",")
                        _anim_file = _anim[0]
                        _blocking = True
                        if len(_anim) > 1:
                            _specified_blocking = _anim[1]
                            _specified_blocking = _specified_blocking.strip()
                            _specified_blocking = _specified_blocking.lstrip()
                            if _specified_blocking == "nb":
                                _blocking = False
                        rospy.loginfo('_blocking = ' + str(_blocking))

                        # The action/animation should block before we play more
                        # speech or do more actions/animations.
                        if _blocking:
                            rospy.loginfo('_anim_file = ' + _anim_file)

                            # Send a lookat game/screen message.
                            if (_anim_file == "lookat-game") or \
                                (_anim_file == "lookat-screen"):
                                # TODO: make these values globally available
                                self.send_jibo_lookat(0.15, -0.65, 0.25)

                            # Using our head tracking info, send a lookat child
                            # command.
                            elif (_anim_file == "lookat_child") or \
                                    (_anim_file == "return-front") or \
                                    (_anim_file == "lookat-child"):
                                rospy.loginfo('user head: ' + str(abs(
                                    self._current_user_head_pos_jibo_rf.x))
                                    + ' ' +
                                    str(self._current_user_head_pos_jibo_rf.y)
                                    + ' ' +
                                    str(self._current_user_head_pos_jibo_rf.z))
                                my_y = self._current_user_head_pos_jibo_rf.y
                                if my_y >= 0.0:
                                    my_y = 0
                                if my_y < -0.4:
                                    my_y = 0
                                self.send_jibo_lookat(abs(
                                    self._current_user_head_pos_jibo_rf.x),
                                    my_y,
                                    self._current_user_head_pos_jibo_rf.z)

                            # Send a lookat guardian command.
                            elif _anim_file == "lookat_guardian":
                                # TODO: need to update these values
                                self.send_jibo_lookat(1, 0.5, 1)

                            # Otherwise, it's a named animation to play back.
                            else:
                                self.send_jibo_animation(_anim_file+'-2.keys')

                        # The action/animation should play at the same time as
                        # the next chunk of speech.
                        # TODO Much the code in this else-block duplicates the
                        # logic and functionality above -- need to consolidate
                        # and find a more consie way to write this! For example,
                        # everything up until sending speech along with the
                        # action could be put in its own function that we call
                        # here, and above...
                        else:
                            # Send a lookat game/screen message.
                            if (_anim_file == "lookat-game") or \
                                (_anim_file == "lookat-screen"):
                                # TODO: make these values globally available
                                self.send_jibo_lookat(0.15, -0.65, 0.25)

                            # Using our head tracking info, send a lookat child
                            # command.
                            elif (_anim_file == "lookat_child") or \
                                    (_anim_file == "return-front") or \
                                    (_anim_file == "lookat-child"):
                                rospy.loginfo('user head: ' + str(abs(
                                    self._current_user_head_pos_jibo_rf.x))
                                    + ' ' +
                                    str(self._current_user_head_pos_jibo_rf.y)
                                    + ' ' +
                                    str(self._current_user_head_pos_jibo_rf.z))
                                my_y = self._current_user_head_pos_jibo_rf.y
                                if my_y >= 0.0:
                                    my_y = 0
                                if my_y < -0.4:
                                    my_y = 0
                                self.send_jibo_lookat(
                                    abs(self._current_user_head_pos_jibo_rf.x),
                                    my_y,
                                    self._current_user_head_pos_jibo_rf.z)

                            # Send a lookat guardian command.
                            elif _anim_file == "lookat_guardian":
                                # TODO: need to update these values
                                self.send_jibo_lookat(1, 0.5, 1)

                            # Otherwise, it's a named animation to play back.
                            else:
                                self.send_jibo_animation(_anim_file+'-2.keys')

                            # Now, if there is speech to play after this action/
                            # animation, we also send a command to play the
                            # speech.
                            if not behavior_queue.empty():
                                # We assume that no two animations are attached
                                # together.
                                _speech = behavior_queue.get()
                                rospy.loginfo('_speech = ' + _speech)
                                self.send_jibo_speech(_speech,
                                    speech_parameters)

                    # There are no animations to send; just send speech!
                    else:
                        #_msg.speech = _content
                        rospy.loginfo('_speech/content = ' + _content)
                        self.send_jibo_speech(_content, speech_parameters)

                    self._is_robot_ready = False
                    self._is_jibo_ready = False
                    # Sleep for 100 ms: do not use 50ms or below.
                    rospy.Rate(10).sleep()

            # Need to wait for jibo to finish the last command request.
            while self._is_jibo_ready == False:
                rospy.Rate(10).sleep()
                continue

            # Send a conceptual robot state signaling that the robot is ready
            # again.
            self._is_robot_ready = True
            conceptual_robot_state = RobotState()
            conceptual_robot_state.header = Header()
            conceptual_robot_state.header.stamp = rospy.Time.now()
            conceptual_robot_state.doing_action = False
            conceptual_robot_state.is_playing_sound = False
            self.robot_state_pub.publish(conceptual_robot_state)


    def send_jibo_lookat(self, _x, _y, _z, _duration=-1):
        """ Build and send Jibo lookat message. """
        msg = JiboLookat()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.x = _x
        msg.y = _y
        msg.z = _z
        msg.duration = _duration
        self.jibo_lookat_pub.publish(msg)


    def send_jibo_animation(self, _name, _n=1.0):
        """ Build and send Jibo animation message. """
        msg = JiboAnimation()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.animation_name = _name
        msg.repeat_n = _n
        self.jibo_animation_pub.publish(msg)


    def send_jibo_speech(self, _content, _parameters):
        """ Build and send a Jibo speech message. """
        msg = JiboSpeech()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.speech_content = _content
        # If we have Jibo speech parameters, use them.
        # TODO Add error checking: what if these keys are not in the dictionary?
        if _parameters:
            msg.pitch = _parameters['pitch']
            msg.pitch_bandwidth = _parameters['pitchBandwidth']
            msg.duration_stretch = _parameters['duration_stretch']
        # Otherwise, use the default paramters for Jibo speech.
        else:
            msg.pitch = 7.6
            msg.pitch_bandwidth = 2.0
            msg.duration_stretch = 1.07
        self.jibo_speech_pub.publish(msg)


    def user_head_position_jibo_rf_callback(self, data):
        """ Receive user head position messages. """
        self._current_user_head_pos_jibo_rf.x = data.x
        self._current_user_head_pos_jibo_rf.y = data.y
        self._current_user_head_pos_jibo_rf.z = data.z


    def send_to_simulated(self, data):
        """ Translate robot command to format simulated robot uses. """
        # Simulated robot just gets standard RobotCommand messages.
        msg = RobotCommand()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # Add message content.
        msg.command = data.command
        msg.properties = data.properties
        # Send message.
        self.robot_sim_pub.publish(msg)
        rospy.loginfo("Forwarding message to simulated robot:\n" + str(msg))


    def send_to_other_robot(self, data):
        """ Translate robot command to format other robot uses. """
        # Fill in to send commands to other robot.
        rospy.logwarn("TODO send to other robot!")


if __name__ == '__main__':
    # Run the node!
    node = robot_translation()
    node.run_robot_translation_node()

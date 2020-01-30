#!/usr/bin/env python

#import re
import rospy
import tf

from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from tf.transformations import euler_from_quaternion


class LaserTest():
    """
    A class used to execute laser tests
    """

    def __init__(self):
        rospy.loginfo("Initialize laser precision test")

        self.executions = [0.0174533, -0.0174533, 0, 0.0872665, -0.0872665, 0,
                           0.174533, -0.174533, 0, 0.349066, -0.349066, 0, 0.523599, -0.523599, 0]

        self.get_pos = False
        self.servo_pos = []
        self.laser_pos = []

        self.servo2_pub = rospy.Publisher(
            '/joint_2/command', Float64, queue_size=10)
        self.servo2_pub.publish(Float64(0.0))

        self.servo1_pub = rospy.Publisher(
            '/joint_1/command', Float64, queue_size=10)
        self.servo1_pub.publish(Float64(0.0))

        self.listener = tf.TransformListener()

        self.servo_sub = rospy.Subscriber(
            "/joint_1/state", JointState, self.servoCallback)

    def start(self):

        rospy.sleep(rospy.Duration(10, 0))

        # Print initial position
        rospy.logwarn("Servo and laser initial pose: %f %f" % (0.0, 0.0))
        # Calculate actual position:
        prev_servo_pos, prev_laser_pos = self.calculateActualPos()

        for pos in self.executions:
            # Move servo to the new position:
            self.servo1_pub.publish(Float64(pos))
            rospy.sleep(rospy.Duration(5, 0))
            # Calculate new position:
            new_servo_pos, new_laser_pos = self.calculateActualPos()
            # Print increment of positions:
            rospy.logwarn("Servo and laser increment: %f %f" % (
                abs(new_servo_pos - prev_servo_pos), abs(new_laser_pos - prev_laser_pos)))

        pass

    def calculateActualPos(self):

        # During 10 seconds, get position values
        self.servo_pos, self.laser_pos = [], []
        self.get_pos = True

        for _ in range(10):
            rospy.sleep(rospy.Duration(1, 0))
            try:
                (_, rot) = self.listener.lookupTransform(
                    'laser_link', 'filtered_docking_station_laser', rospy.Time(0))
                (_, _, yaw) = euler_from_quaternion(rot)
                self.laser_pos.append(yaw)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.get_pos = False

        act_servo_pos = sum(self.servo_pos) / len(self.servo_pos)
        act_laser_pos = sum(self.laser_pos) / len(self.laser_pos)

        return act_servo_pos, act_laser_pos

    def servoCallback(self, data):
        if self.get_pos == True:
            self.servo_pos.append(data.current_pos)

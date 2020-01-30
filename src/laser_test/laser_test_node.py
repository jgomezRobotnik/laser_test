#!/usr/bin/env python

import rospy

from laser_test import LaserTest


def main():

    rospy.init_node("laser_test")

    lt_node = LaserTest()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    lt_node.start()


if __name__ == "__main__":
    main()

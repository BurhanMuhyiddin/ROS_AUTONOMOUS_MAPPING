#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

class FollowWall():
    def __init__(self):
        rospy.init_node('wall_follower')

        self.regions_ = {
                'right': 0,
                'fright': 0,
                'front': 0,
                'fleft': 0,
                'left': 0,
        }
        self.state_ = 0
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }

        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)

        self.handle_cases()

    def handle_cases(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            msg = Twist()
            if self.state_ == 0:
                msg = self.find_wall()
            elif self.state_ == 1:
                msg = self.turn_left()
            elif self.state_ == 2:
                msg = self.follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            self.pub_.publish(msg)

            rate.sleep()

    def change_state(self, state):
        if state is not self.state_:
            print ('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state

    def clbk_laser(self, msg):
        self.regions_ = {
            'left':  min(min(msg.ranges[56:92]), 3.5),
            'fleft': min(min(msg.ranges[19:55]), 3.5),
            'front':  min(min(msg.ranges[341:359]), min(msg.ranges[0:18]), 3.5),
            'fright':  min(min(msg.ranges[304:340]), 3.5),
            'right':   min(min(msg.ranges[267:303]), 3.5),
        }

        self.take_action()

    def take_action(self):
        regions = self.regions_
        msg = Twist()
        linear_x = 0
        angular_z = 0

        state_description = ''

        d = 0.8

        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.3
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.3
        return msg


if __name__ == "__main__":
    try:
        FollowWall()
    except rospy.ROSInterruptException:
        rospy.loginfo("Following finished.")

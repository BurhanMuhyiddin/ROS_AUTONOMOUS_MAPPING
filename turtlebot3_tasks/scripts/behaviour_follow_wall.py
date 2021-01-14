#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import *

import math

class FollowWall():
    def __init__(self):
        rospy.init_node('follow_wall_node')

        self.d = 0.4
        self.regions_ = {
                'fright': 0,
                'front': 0,
                'fleft': 0
        }
        self.state_ = 0
        self.active_ = False
        self.state_dict_ = {
            0 : 'correct head',
            1 : 'correct side',
            2 : 'correct behaviour'
        }

        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.srv = rospy.Service("follow_wall_switch", SetBool, self.follow_wall_switch)

        self.handle_cases()

    def follow_wall_switch(self, req):
        self.active_ = req.data
        res = SetBoolResponse()
        res.success = True
        return res

    def handle_cases(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.active_:
                rate.sleep()
                continue
            else:
                msg = Twist()

                if self.state_ == 0:
                    msg = self.correct_head()
                elif self.state_ == 1:
                    msg = self.correct_side()
                elif self.state_ == 2:
                    msg = self.correct_behaviour()
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
            'fleft': min(min(msg.ranges[19:55]), 3.5),
            'front':  min(min(msg.ranges[341:359]), min(msg.ranges[0:18]), 3.5),
            'fright':  min(min(msg.ranges[304:340]), 3.5)
        }

        self.take_action()

    def take_action(self):
        regions = self.regions_
        msg = Twist()
        linear_x = 0
        angular_z = 0

        if regions['front'] < self.d:
            self.change_state(0) # stop and turn left untill front is empty
        else:
            if regions['fleft'] < self.d and regions['fright'] >= self.d:
                self.change_state(2)
            else:
                self.change_state(1) # turn pd to keep threshold

    def correct_head(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.2
        return msg

    def correct_side(self):
        Kp = 0.8
        msg = Twist()
        msg.linear.x = 0.15
        error = self.d - self.regions_['fright']
        if abs(error) > 1.0:
           if error < 0:
               error = -1.0
           else:
               error = 1.0
        msg.angular.z = Kp * ( error )
        return msg

    def correct_behaviour(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        return msg


if __name__ == "__main__":
    try:
        FollowWall()
    except rospy.ROSInterruptException:
        rospy.loginfo("Following finished.")

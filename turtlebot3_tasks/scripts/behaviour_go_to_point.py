#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import *
from tf.transformations import euler_from_quaternion

import math
import time

class GoToPoint:
    def __init__(self):
        rospy.init_node("go_to_goal_node")
        rospy.Subscriber("/odom", Odometry, self.get_odom_data)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.srv = rospy.Service("go_to_point_switch", SetBool, self.go_to_point_switch)
        self.robot_crd = {
            "x"  : 0.0,
            "y"  : 0.0,
            "th" : 0.0
        }
        self.active_ = False

        self.go_to_point()

    def go_to_point_switch(self, req):
        self.active_ = req.data
        res = SetBoolResponse()
        res.success = True
        return res

    def go_to_point(self):
        dist = 1
        integral_term = 0.0
        current_time = time.time()
        last_time    = current_time

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.active_:
                rate.sleep()
                continue
            else:
                if(dist >= 0.05):
                    # get desired and robot coordinates
                    d_crd = [ rospy.get_param("desired_x"), rospy.get_param("desired_y") ]
                    r_crd = [ self.robot_crd["x"], self.robot_crd["y"], self.robot_crd["th"] ]

                    current_time = time.time()
                    delta_time = current_time - last_time

                    # calculate heading angle
                    Kh = 0.5
                    des_ang = math.atan2(d_crd[1] - r_crd[1], d_crd[0] - r_crd[0])
                    heading_ang = Kh * (des_ang - r_crd[2])

                    # calculate heading velocity
                    Kv = 0.04
                    Ki = 0.0007
                    integral_term = integral_term + dist * delta_time
                    dist = math.sqrt( (d_crd[0] - r_crd[0]) * (d_crd[0] - r_crd[0]) +
                                      (d_crd[1] - r_crd[1]) * (d_crd[1] - r_crd[1]) )
                    heading_vel = Kv * dist + Ki * integral_term

                    # publish data
                    new_twist_cmd = Twist()
                    new_twist_cmd.linear.x  = heading_vel
                    new_twist_cmd.angular.z = heading_ang
                    self.twist_pub.publish(new_twist_cmd)

                    last_time = current_time

                else:
                    new_twist_cmd = Twist()
                    new_twist_cmd.linear.x  = 0
                    new_twist_cmd.linear.y  = 0
                    new_twist_cmd.angular.z = 0
                    self.twist_pub.publish(new_twist_cmd)
                rate.sleep()

    def get_odom_data(self, msg):
        # get yaw
        ornt_quaternion = msg.pose.pose.orientation
        temp = [ ornt_quaternion.x, ornt_quaternion.y, ornt_quaternion.z, ornt_quaternion.w ]
        (roll, pitch, yaw) = euler_from_quaternion(temp)

        self.robot_crd["x"] = msg.pose.pose.position.x
        self.robot_crd["y"] = msg.pose.pose.position.y
        self.robot_crd["th"] = yaw

if __name__ == '__main__':
    GoToPoint()

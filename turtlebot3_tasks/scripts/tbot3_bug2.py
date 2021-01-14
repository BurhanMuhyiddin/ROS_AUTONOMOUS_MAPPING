#!/usr/bin/env python

import rospy
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import math

class BugTwo:
    def __init__(self):
        rospy.init_node("bugtwo_node")

        rospy.wait_for_service("go_to_point_switch")
        rospy.wait_for_service("follow_wall_switch")

        rospy.Subscriber("/odom", Odometry, self.get_odom_data)
        rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.service_client_go_to_point = rospy.ServiceProxy("go_to_point_switch", SetBool)
        self.service_client_follow_wall = rospy.ServiceProxy("follow_wall_switch", SetBool)

        self.y_range = [0.0, 0.0]
        self.state = 0 # start with go to point mode
        self.states = {
            0 : "go_to_point",
            1 : "follow_wall"
        }
        self.slope = 0.0
        self.b = 0.0
        self.d_crd = {
            "x" : rospy.get_param("desired_x"),
            "y" : rospy.get_param("desired_y")
        }
        self.r_crd = {
            "x"  : 0.0,
            "y"  : 0.0,
            "th" : 0.0
        }
        self.regions_ = {
            "fright" : 0,
            "front"  : 0
        }
        self.last_state = 0

        self.bug_two()

    def clbk_laser(self, msg):
        self.regions_ = {
            'fleft': min(min(msg.ranges[19:55]), 3.5),
            'front' :  min(min(msg.ranges[341:359]), min(msg.ranges[0:18]), 3.5),
            'fright':  min(min(msg.ranges[304:340]), 3.5),
        }

        #self.take_action()

    def get_odom_data(self, msg):
        ornt_quaternion = msg.pose.pose.orientation
        temp = [ ornt_quaternion.x, ornt_quaternion.y, ornt_quaternion.z, ornt_quaternion.w ]
        (roll, pitch, yaw) = euler_from_quaternion(temp)

        self.r_crd["x"]  = msg.pose.pose.position.x
        self.r_crd["y"]  = msg.pose.pose.position.y
        self.r_crd["th"] = yaw

    def get_equation_of_line(self):
        self.y_range = [min(self.d_crd["y"], self.r_crd["y"]), max(self.d_crd["y"], self.r_crd["y"])]
        x_delta = (self.d_crd["x"] - self.r_crd["x"])
        if x_delta != 0.0:
            self.slope = (self.d_crd["y"] - self.r_crd["y"]) / x_delta
            self.b = -1.0 * self.slope * self.r_crd["x"] + self.r_crd["y"]

    def change_state(self, st):
        self.last_state = self.state
        self.state = st
        print( "State changed to: [%s]..." % (self.states[st]) )
        if self.state == 0:
            self.service_client_go_to_point(True)
            self.service_client_follow_wall(False)
        elif self.state == 1:
            self.service_client_go_to_point(False)
            self.service_client_follow_wall(True)
            if self.last_state == 0:
                rospy.sleep(10.0)

    def is_on_the_line(self):
        y = self.slope * self.r_crd["x"] + self.b
        if self.y_range[0] <= y <= self.y_range[1]:
            err = abs(y - self.r_crd["y"])
            #print("line error: " + str(err))
            if err <= 0.01:
                return True
        return False

    def take_action(self):
        if self.state == 0 and self.regions_["front"] < 0.6:
            self.change_state(1) # change state to follow wall mode
        elif self.state == 1 and self.is_on_the_line():
            self.service_client_follow_wall(False)
            # stop robot
            self.stop_robot()
            # turn towards the goal
            self.correct_head_angle()
            # check if there is obstackle
            if self.regions_["front"] < 0.6:
                # if yes follow wall mode
                self.change_state(1)
            # else go to point mode
            else:
                self.change_state(0) # change state to go to point mode

    def stop_robot(self):
        print("Stopping robot...")
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.pub_.publish(vel)

    def correct_head_angle(self):
        print("Correcting head angle...")
        vel = Twist()
        while True:
            Kh = 1.0
            des_ang = math.atan2(self.d_crd["y"] - self.r_crd["y"], self.d_crd["x"] - self.r_crd["x"])
            err = des_ang - self.r_crd["th"]
            print("heading error: " + str(err) )
            heading_ang = Kh * err * (-1)
            if (abs(err) <= 0.0006):
                break
            vel.angular.z = heading_ang
            self.pub_.publish(vel)
        vel.angular.z = 0.0
        self.pub_.publish(vel)

    def bug_two(self):
        rate = rospy.Rate(20)
        self.change_state(0)
        rospy.sleep(5.0)
        self.get_equation_of_line()
        while not rospy.is_shutdown():
            self.take_action()
            #print(self.regions_["front"])
            rate.sleep()

if __name__ == '__main__':
    BugTwo()

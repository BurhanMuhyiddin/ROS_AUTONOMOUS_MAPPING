#! /usr/bin/env python

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class SetNavGoals():
    def __init__(self):
        rospy.init_node("navigation_goals_setter")
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped)

        ps = Pose()
        self.pos = ps.position;
        self.ori = ps.orientation;
        self.flag = False
        self.is_published = False

        self.pose_seq = list()
        self.goal_cnt = 0
        self.set_initial_pose()
        self.get_goal_points()

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def set_initial_pose(self):
        target_topic = ['/initialpose', 'geometry_msgs/PoseWithCovarianceStamped']
        topics = rospy.get_published_topics()
        while not (target_topic in topics):
            topics = rospy.get_published_topics()
        while not self.flag:
            continue
        initial_point=PoseWithCovarianceStamped()
        initial_point.header.frame_id = "map"
        initial_point.header.stamp =rospy.Time(0)
        initial_point.pose.pose.position = self.pos
        initial_point.pose.pose.orientation = self.ori
        tmr = rospy.Timer(rospy.Duration(3), self.stop_publishing_callback)
        rospy.loginfo("Initial pose of the robot is being set...")
        while not self.is_published:
            self.pub.publish(initial_point)
        rospy.loginfo("Initial pose of the robot has been set.")
        tmr.shutdown()

    def get_goal_points(self):
        points_seq = list()
        eul_angle_seq = list()
        quat_seq = list()

        state = "c_cords"
        f = open("goals.txt", "r")
        while 1:
            line = f.readline()
            if line == "---\n":
                state = "o_cords"
                continue
            if line == "":
                break
            if state == "c_cords":
                c_cords = line.split(",")
                for c_cord in c_cords:
                    points_seq.append(float(c_cord))
            elif state == "o_cords":
                eul_angle_seq.append(float(line))

        for eul_angle in eul_angle_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, eul_angle*math.pi/180, axes='sxyz'))))
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

    def odom_callback(self, data):
        self.flag = True
        self.pos = data.pose.pose.position
        self.ori = data.pose.pose.orientation

    def stop_publishing_callback(self, event):
        self.is_published = True

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        SetNavGoals()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

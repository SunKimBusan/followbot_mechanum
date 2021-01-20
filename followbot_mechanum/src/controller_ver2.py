#!/usr/bin/env python
import rospy

from std_msgs.msg import String, Float64MultiArray
from leg_tracker.msg import PersonArray
import sys

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, degrees, pow, sqrt
import time
import tf2_ros
import tf2_geometry_msgs

from tf import transformations


#person_name = sys.argv[1]
person_name = 'taeyang'
min_effective_distance = 0.4 # m
max_effective_distance = 5 # m
global rotation_cnt
rotation_cnt = 0

class Controller():
    def __init__(self):

        self.face_auth = False
        self.moving_mode = False
        self.tracked_person = None
        self.odom = None
        self.leg_data = None
        self.personPose = None
        self.start_time = None

        self.face_name_sub = rospy.Subscriber('face_name', String, self.callback_face_name)
        self.legs_sub = rospy.Subscriber('people_tracked', PersonArray, self.callback_legs)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Tell the action client that we want to spin a thread by default
    	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    	rospy.loginfo("Wait for the action server to come up")
        # Allow up to 5 seconds for the action server to come up
    	self.move_base.wait_for_server(rospy.Duration(5))

        self.personPose_sub = rospy.Subscriber('person_pose', Float64MultiArray, self.personPoseCallback)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def personPoseCallback(self,data):

        self.start_time = time.time()

        if data.data[2] != 0 and data.data[0] != 0:

            self.personPose = data.data

    def callback_odom(self,data):
        self.odom = data

    def callback_face_name(self,data):
        name = data.data.split('_')[0]
        self.face_auth = (name == person_name)

    def callback_legs(self,data):

        self.leg_data = data

    def move2goal(self, constant=6):

        r = rospy.Rate(2)

        while not rospy.is_shutdown():

            print('---- moving ----')

            if self.personPose is None :
                continue

            time_interval =  self.start_time - time.time()

            if time_interval > 10:
                print('Target disappeared')
                self.moving_mode = False
                break

            odom_msg = self.odom

            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            rot_q = odom_msg.pose.pose.orientation

            (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

            speed = Twist()

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'camera_link'
            pose_stamped.pose.position.x = self.personPose[2]/1000
            pose_stamped.pose.position.y = self.personPose[0]/1000
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            pose_stamped.header.stamp = rospy.Time.now()

            camera_transform = self.tf_buffer.lookup_transform('odom', 'camera_link', rospy.Time(0), rospy.Duration(1.0)) #wait for 1 second
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, camera_transform)

            inc_x = pose_transformed.pose.position.x - x
            inc_y = pose_transformed.pose.position.y - y

            angle_to_goal = atan2(inc_y, inc_x)

            euclidean_distance = sqrt(pow((inc_x), 2) + pow((inc_y), 2))
            if euclidean_distance < 0.3:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
            else:

                x_vel = 1.5 * euclidean_distance
                if x_vel < 3:
                    speed.linear.x = x_vel
                else:
                    speed.linear.x = 3.0

                z_vel = 6 * (angle_to_goal - yaw)
                if z_vel < 0.9:
                    speed.angular.z = z_vel
                else:
                    speed.angular.z = 0.9

            self.cmd_vel_pub.publish(speed)

            r.sleep()

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)


    def detecting_target_person(self):

        global rotation_cnt
        rotation_cnt = rotation_cnt + 1

        print('===callback_legs===\n')
        local_leg_data = self.leg_data

        for person in local_leg_data.people:
            if self.rotate_to_check_target(person):
                # initialize auth before rotation.
                self.face_auth = False
                rospy.sleep(2)

                if self.face_auth:
                    print("self.face_auth - ", self.face_auth)
                    self.moving_mode = True
                    break

        print('===callback_legs end===')

    def rotate_to_check_target(self,person):

        global rotation_cnt

        distance = person.pose.position.x**2 + person.pose.position.y**2
        if distance > max_effective_distance**2 or distance < min_effective_distance**2 :
            print("beyond effective distance id - ", person.id)
            return False

        latest_leg_data = self.leg_data
        id_exist_in_latest = False
        for latest_person in latest_leg_data.people:
            if latest_person.id == person.id:
                id_exist_in_latest = True
                break
        if id_exist_in_latest == False:
            return False

        r = rospy.Rate(2)

        while not rospy.is_shutdown():
            print("rotation_cnt : ", rotation_cnt)
            print("Rotating to id : ", person.id)
            print("")
            print("")
            print("")

            if self.odom is None :
                continue

            odom_msg = self.odom

            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            rot_q = odom_msg.pose.pose.orientation

            (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

            speed = Twist()

            inc_x = person.pose.position.x - x
            inc_y = person.pose.position.y - y

            angle_to_goal = atan2(inc_y, inc_x)

            print('degree - angle_to_goal : ', degrees(angle_to_goal))
            print('degree - yaw : ', degrees(yaw))
            print('angle_to_goal - yaw :', degrees(angle_to_goal - yaw))
            print("")
            print("")
            print("---------------------------------------------")

            if abs(angle_to_goal - yaw) > 0.1:
                speed.linear.x = 0.0
                if angle_to_goal > 0 and yaw < 0:
                    if angle_to_goal + abs(yaw) > 3.14:
                        speed.angular.z = -0.3
                    else:
                        speed.angular.z = 0.3
                elif angle_to_goal < 0 and yaw > 0:
                    if abs(angle_to_goal) + yaw > 3.14:
                        speed.angular.z = 0.3
                    else:
                        speed.angular.z = -0.3
                elif (angle_to_goal - yaw) > 0:
                    speed.angular.z = 0.3
                else:
                    speed.angular.z = -0.3
                self.cmd_vel_pub.publish(speed)
            else:
                print("---- Complete Rotating ----")
                print("---- Complete Rotating ----")
                print("---- Complete Rotating ----")
                break

            r.sleep()

        return True

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=False)
    controller  = Controller()

    r = rospy.Rate(2)

    while not rospy.is_shutdown():

        if controller.moving_mode == False:
            controller.detecting_target_person()
        else:
            controller.move2goal()
        r.sleep()

    rospy.spin()

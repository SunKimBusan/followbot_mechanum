#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from followbot_turtle.msg import Clothes
from leg_tracker.msg import PersonArray
import sys

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2

#person_name = sys.argv[1]
person_name = 'taeyang'
min_effective_distance = 0.2 # m
max_effective_distance = 5 # m

class Controller():
    def __init__(self):

        self.face_auth = False
        self.clothes_auth = False
        self.detecting_target = False
        self.moving_mode = False
        self.is_moving = False
        self.tracked_person = None
        self.odom = None
        self.leg_data = None

        self.face_name_sub = rospy.Subscriber('face_name', String, self.callback_face_name)
        self.clothes_sub = rospy.Subscriber('clothes', Clothes, self.callback_clothes)
        self.legs_sub = rospy.Subscriber('people_tracked', PersonArray, self.callback_legs)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Tell the action client that we want to spin a thread by default
    	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    	rospy.loginfo("Wait for the action server to come up")
        # Allow up to 5 seconds for the action server to come up
    	self.move_base.wait_for_server(rospy.Duration(5))


    def callback_odom(self,data):
        self.odom = data

    def callback_face_name(self,data):
        name = data.data.split('_')[0]
        self.face_auth = (name == person_name)

    def callback_clothes(self,data):
        name = data.name.split('_')[0]
        self.clothes_auth = (name == person_name)
        #print('clothes_auth : ', self.clothes_auth, ' sim : ', data.similarity)

    def callback_legs(self,data):

        self.leg_data = data

        if self.moving_mode == False:
            if self.detecting_target == False:
                self.detecting_target_person()
        else:
            if self.is_moving == False:
                self.checking_target()

    def checking_target(self):

        self.is_moving = True

        local_leg_data = self.leg_data
        print('Targeting id : ', self.tracked_person.id)

        there_is_target_id = False

        for person in local_leg_data.people:
            print('id - ', person.id)
            if person.id == self.tracked_person.id:
                self.tracked_person = person
                there_is_target_id = True
                break

        if there_is_target_id == False :
            print('Target disappeared')
            self.moving_mode = False
            self.tracked_person = None
            self.is_moving = False
            return

        r = rospy.Rate(4)

        cnt = 0

        while not rospy.is_shutdown():

            if cnt == 12:
                break

            person = self.tracked_person
            odom_msg = self.odom
            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            rot_q = odom_msg.pose.pose.orientation
            (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(person.pose.position.x, person.pose.position.y, person.pose.position.z),
                Quaternion(person.pose.orientation.x, person.pose.orientation.y, person.pose.orientation.z, person.pose.orientation.w))

            speed = Twist()

            inc_x = goal.target_pose.pose.position.x - x
            inc_y = goal.target_pose.pose.position.y - y

            angle_to_goal = atan2(inc_y, inc_x)

            if abs(angle_to_goal - theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.6
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0

            self.cmd_vel_pub.publish(speed)
            print('---moving---')
            cnt += 1

            r.sleep()

        self.is_moving = False


    def detecting_target_person(self):

        print('===callback_legs===\n')
        local_leg_data = self.leg_data
        self.detecting_target = True

        for person in local_leg_data.people:
            if self.rotate_to_check_target(person):
                # initialize auth before rotation.
                self.face_auth = False
                self.clothes_auth = False
                rospy.sleep(2)

                if self.face_auth or self.clothes_auth:
                    print("self.face_auth - ", self.face_auth)
                    print("self.clothes_auth - ", self.clothes_auth)
                    self.tracked_person = person
                    self.moving_mode = True
                    break

        self.detecting_target = False
        print('===callback_legs end===')

    def rotate_to_check_target(self,person):

        distance = person.pose.position.x**2 + person.pose.position.y**2
        if distance > max_effective_distance**2 or distance < min_effective_distance**2 :
            print("beyond effective distance id - ", person.id)
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(person.pose.position.x, person.pose.position.y, person.pose.position.z),
            Quaternion(person.pose.orientation.x, person.pose.orientation.y, person.pose.orientation.z, person.pose.orientation.w))



        r = rospy.Rate(2)

        while not rospy.is_shutdown():
            print("Rotating to id : ", person.id)

            if self.odom is None :
                continue

            odom_msg = self.odom
            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            rot_q = odom_msg.pose.pose.orientation

            (roll, pitch, theta) = euler_from_quaternion([rot_q .x, rot_q.y, rot_q.z, rot_q.w])

            speed = Twist()

            inc_x = goal.target_pose.pose.position.x - x
            inc_y = goal.target_pose.pose.position.y - y

            angle_to_goal = atan2(inc_y, inc_x)

            print(angle_to_goal - theta)

            if abs(angle_to_goal - theta) > 0.1:
                #print('rotate - rotate_to_check_target')
                speed.linear.x = 0.0
                speed.angular.z = 0.3
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
    rospy.spin()

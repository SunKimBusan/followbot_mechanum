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
from math import atan2, degrees, pow, sqrt

#person_name = sys.argv[1]
person_name = 'taeyang'
min_effective_distance = 0.4 # m
max_effective_distance = 4 # m
global rotation_cnt
rotation_cnt = 0

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

    def move2goal(self, constant=6):

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

        r = rospy.Rate(2)

        cnt = 0

        while not rospy.is_shutdown():

            if cnt == 12:
                break

            person = self.tracked_person
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

            cnt += 1
            r.sleep()

        self.is_moving = False


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
            if controller.detecting_target == False:
                controller.detecting_target_person()
        else:
            if controller.is_moving == False:
                controller.move2goal()
        r.sleep()

    rospy.spin()

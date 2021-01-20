#!/usr/bin/env python
import rospy

from std_msgs.msg import String, Float64MultiArray
import sys
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, degrees, pow, sqrt
import time
import tf2_ros
import tf2_geometry_msgs
from tf import transformations
from sensor_msgs.msg import LaserScan
import numpy as np

from visualization_msgs.msg import Marker

class Controller():
    def __init__(self):

        self.moving_mode = False
        self.odom = None
        self.personPose = None
        self.pose_transformed = None
        self.start_time = None
        self.obstacle_mode = False
        self.is_arrived = False

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.personPose_sub = rospy.Subscriber('person_pose', Float64MultiArray, self.personPoseCallback)
        self.marker_pub = rospy.Publisher('person_marker', Marker, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)

    def callback_laser(self,msg):
        # Using RPLIDAR s1
        regions = {
            'front_left':  min(min(msg.ranges[0:92]), 10),
            'front_right':  min(min(msg.ranges[664:720]), 10),
            'right':  min(min(msg.ranges[463:617]), 10),
        }

        self.take_action(regions)

    def take_action(self,regions):
        msg = Twist()
        linear_x = 0
        angular_z = 0
        threshold = 0.5

        state_description = ''

        is_obstacle = False

        if regions['front_left'] < threshold or regions['front_right'] < threshold:
            state_description = 'front obstacle'
            linear_x = 0
            angular_z = 0.3
            is_obstacle = True
        elif regions['right'] < threshold:
            state_description = 'right obstacle'
            linear_x = 0.3
            angular_z = 0
            is_obstacle = True

        self.obstacle_mode = is_obstacle

        if self.obstacle_mode and not self.is_arrived and self.moving_mode:
            rospy.loginfo(state_description)
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.cmd_vel_pub.publish(msg)

    def personPoseCallback(self,data):


        self.start_time = time.time()

        if data.data[2] != 0 and data.data[0] != 0:
            self.personPose = data.data
            print('---- renew person position ----')

            odom_msg = self.odom

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'camera_link'
            # personPose : [-y, x, z] unit : mm
            pose_stamped.pose.position.x = self.personPose[1]/1000
            pose_stamped.pose.position.y = -self.personPose[0]/1000
            pose_stamped.pose.position.z = self.personPose[2]/1000
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            pose_stamped.header.stamp = rospy.Time.now()

            camera_transform = self.tf_buffer.lookup_transform('odom', 'camera_link', rospy.Time(0), rospy.Duration(1.0)) #wait for 1 second
            self.pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, camera_transform)

            ## send marker
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.pose_transformed.pose.position.x
            marker.pose.position.y = self.pose_transformed.pose.position.y
            marker.pose.position.z = 0.0

            self.marker_pub.publish(marker)

            self.moving_mode = True


    def callback_odom(self,data):
        self.odom = data

    def move2goal(self, constant=6):

        print('---- moving ----')

        r = rospy.Rate(3)

        while not rospy.is_shutdown():

            time_interval =  time.time() - self.start_time
            print("time_interval - ", time_interval)

            if time_interval > 20:
                print('---- Target disappeared ----')
                self.moving_mode = False
                result = self.rotation()
                if result == False:
                    break
                print('---- moving ----')

            if self.obstacle_mode == True:
                r.sleep()
                continue

            odom_msg = self.odom

            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            rot_q = odom_msg.pose.pose.orientation

            (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

            speed = Twist()

            inc_x = self.pose_transformed.pose.position.x - x
            inc_y = self.pose_transformed.pose.position.y - y

            #print(pose_transformed)

            angle_to_goal = atan2(inc_y, inc_x)


            '''
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

            self.cmd_vel_pub.publish(speed)'''


            euclidean_distance = sqrt(pow((inc_x), 2) + pow((inc_y), 2))
            if euclidean_distance < 1 and abs(angle_to_goal - yaw) <= 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self.is_arrived = True
                print('------ arrived ------')
            else:
                self.is_arrived = False
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
                    speed.linear.x = 0.3
                    speed.angular.z = 0.0

            self.cmd_vel_pub.publish(speed)

            r.sleep()

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def rotation(self):

        start_time = time.time()

        r = rospy.Rate(3)
        while not rospy.is_shutdown():

            if self.moving_mode == True:
                return True

            time_interval =  time.time() - start_time
            if time_interval > 20:
                return False

            speed = Twist()
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            self.cmd_vel_pub.publish(speed)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=False)
    controller  = Controller()

    r = rospy.Rate(3)

    while not rospy.is_shutdown():

        if controller.moving_mode == True:
            controller.move2goal()
        r.sleep()

    rospy.spin()

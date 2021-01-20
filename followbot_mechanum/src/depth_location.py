#!/usr/bin/env python
import rospy
from openpose_ros_msgs.msg import OpenPoseHumanList
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import sys
import cv2
import numpy as np
import pyrealsense2 as rs2
from std_msgs.msg import Float64MultiArray

class depth_location:

	def __init__(self):
		self.bridge = CvBridge()
		self.depth_img = None
		self.intrinsics = None

		self.humanlist_sub = rospy.Subscriber('/openpose_ros/human_list', OpenPoseHumanList, self.callback_humanlist)
		self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.convert_depth_image)
		self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
		self.personPose_pub = rospy.Publisher('person_pose', Float64MultiArray, queue_size=10)


	def callback_humanlist(self,data):

		for human in data.human_list:

			body_bounding_box = human.body_bounding_box
			col = int(round(body_bounding_box.x + body_bounding_box.width/2))
			row = int(round(body_bounding_box.y + body_bounding_box.height/2))
			
			if self.intrinsics is not None:
				if self.depth_img is not None:
					depth = self.depth_img[row, col]
					#print('result : ', result)
					#print(type(result))
					# result = [-y, -z, x] unit : mm
					data_to_send = Float64MultiArray()
					data_to_send.data = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], depth) # [col, row]
					self.personPose_pub.publish(data_to_send)

	def convert_depth_image(self, data):
		try:
			# shape - (480, 640)
			self.depth_img = self.bridge.imgmsg_to_cv2(data, data.encoding)
		except CvBridgeError as e:
			print(e)

	def imageDepthInfoCallback(self, cameraInfo):
		try:
			if self.intrinsics:
				return
			self.intrinsics = rs2.intrinsics()
			self.intrinsics.width = cameraInfo.width
			self.intrinsics.height = cameraInfo.height
			self.intrinsics.ppx = cameraInfo.K[2]
			self.intrinsics.ppy = cameraInfo.K[5]
			self.intrinsics.fx = cameraInfo.K[0]
			self.intrinsics.fy = cameraInfo.K[4]
			if cameraInfo.distortion_model == 'plumb_bob':
				self.intrinsics.model = rs2.distortion.brown_conrady
			elif cameraInfo.distortion_model == 'equidistant':
				self.intrinsics.model = rs2.distortion.kannala_brandt4
				self.intrinsics.coeffs = [i for i in cameraInfo.D]
		except CvBridgeError as e:
			print(e)
			return


def main(args):
	rospy.init_node('depth_location', anonymous=True)
	dl = depth_location()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)

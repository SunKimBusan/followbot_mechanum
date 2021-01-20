#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import sys
import os
import cv2
import numpy as np
import pyrealsense2 as rs2
from std_msgs.msg import Float64MultiArray
import rospkg
import face_recognition

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('followbot_mechanum')
person_name = 'taeyang'

class depth_location:
	def __init__(self):
		self.bridge = CvBridge()
		self.depth_img = None
		self.intrinsics = None
		self.known_face_encodings = []
		self.known_face_names = []

		# Initialize some variables
		self.face_locations = []
		self.face_encodings = []
		self.face_names = []
		self.cv_image = None

		# Load sample pictures and learn how to recognize it.
		dirname = pkg_path + '/src/knowns'
		files = os.listdir(dirname)
		for filename in files:
			name, ext = os.path.splitext(filename)
			if ext == '.jpg':
				self.known_face_names.append(name)
				pathname = os.path.join(dirname, filename)
				img = face_recognition.load_image_file(pathname)
				face_encoding = face_recognition.face_encodings(img)[0]
				self.known_face_encodings.append(face_encoding)

		self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.convert_depth_image)
		self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
		self.personPose_pub = rospy.Publisher('person_pose', Float64MultiArray, queue_size=10)
		self.img_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback_img)

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

	def face_check(self):

		frame = self.cv_image
		depth_img = self.depth_img

		# Find all the faces and face encodings in the current frame of video
		self.face_locations = face_recognition.face_locations(frame)
		self.face_encodings = face_recognition.face_encodings(frame, self.face_locations)

		self.face_names = []
		for face_encoding in self.face_encodings:
			# See if the face is a match for the known face(s)
			distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
			min_value = min(distances)

			# tolerance: How much distance between faces to consider it a match. Lower is more strict.
			# 0.6 is typical best performance.
			name = "Unknown"

			if min_value < 0.6:
				index = np.argmin(distances)
				name = self.known_face_names[index]
				self.face_names.append(name)

		# Display the results
		for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):

			col = int(round((left + right)/2))
			row = int(round((bottom + top)/2))
			face_depth_arr = depth_img[top:bottom,left:right]

			min_depth = 100000
			for i in range(0, len(face_depth_arr)):
				for j in range(0, len(face_depth_arr[0])):
					if face_depth_arr[i][j] != 0:
						if face_depth_arr[i][j] < min_depth:
							min_depth = face_depth_arr[i][j]


			print('col : ', col, 'row : ', row)

			if name.split('_')[0] == person_name :
				if self.intrinsics is not None:
					if self.depth_img is not None:
						#depth = depth_img[row, col]
						print(min_depth)
						data_to_send = Float64MultiArray()
						# personPose : [-y, x, z] by camera_link frame unit : mm
						data_to_send.data = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], min_depth) # [col, row]
						print('data_to_send : ', data_to_send)
						self.personPose_pub.publish(data_to_send)

			# Draw a box around the face
			cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
			# Draw a label with a name below the face
			cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
			font = cv2.FONT_HERSHEY_DUPLEX
			cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

		return frame


	def callback_img(self,data):
		try:
			#### direct conversion to CV2 ####
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			frame = self.face_check()
			cv2.imshow("Frame", frame)
			#cv2.imshow("Frame", self.cv_image)
			cv2.waitKey(1)
			#key = cv2.waitKey(1) & 0xFF

		except CvBridgeError as e:
			print(e)


def main(args):
	rospy.init_node('depth_location', anonymous=True)
	dl = depth_location()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)

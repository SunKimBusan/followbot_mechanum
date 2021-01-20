#!/usr/bin/env python
import rospy

from openpose_ros_msgs.msg import OpenPoseHumanList
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sys
import cv2
from matplotlib import pyplot as plt
import numpy as np
from PIL import Image as pil_image
import glob
from followbot_turtle.msg import Clothes

class clothes_pattern:

	def __init__(self):
		self.bridge = CvBridge()
		self.person_clothes_hists = []
		self.person_names = []
		for filename in glob.glob('person_clothes_img/*.png'):
			im = pil_image.open(filename)
			im_cv = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
			hist = cv2.calcHist([im_cv],[0,1,2],None,[8,8,8],[0,256,0,256,0,256])
			hist = cv2.normalize(hist, hist).flatten()
			self.person_names.append(filename.split('/')[1])
			self.person_clothes_hists.append(hist)
			#print(sum_list(hist))

		self.humanlist_sub = rospy.Subscriber('/openpose_ros/human_list', OpenPoseHumanList, self.callback_humanlist)
		self.img_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback_img)
		self.cv_image = None
		self.clothes_pub = rospy.Publisher('clothes', Clothes, queue_size=10)

	def callback_humanlist(self,data):

		for human in data.human_list:
			body_bounding_box = human.body_bounding_box
			x = int(round(body_bounding_box.x))
			y = int(round(body_bounding_box.y))
			width = int(round(body_bounding_box.width))
			height = int(round(body_bounding_box.height))
			clothes_block = self.cv_image[y:y+height, x:x+width]

			cv2.imshow('Image', clothes_block)
			cv2.waitKey(1)

			hist = cv2.calcHist([clothes_block],[0,1,2],None,[8,8,8],[0,256,0,256,0,256])
			hist = cv2.normalize(hist, hist).flatten()

			print('--------------------')

			for idx,val in enumerate(self.person_clothes_hists):
				# HISTCMP_INTERSECT = 2
				similarity = cv2.compareHist(hist,val,2)
				print(self.person_names[idx])
				print(similarity)
				msg = Clothes()
				if similarity > 3 :
					msg.name = self.person_names[idx]
					msg.similarity = similarity
					self.clothes_pub.publish(msg)

	def callback_img(self,data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

def main(args):
	rospy.init_node('clothes_pattern', anonymous=True)
	cp = clothes_pattern()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)

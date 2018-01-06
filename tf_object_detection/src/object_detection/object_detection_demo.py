#!/usr/bin/env python

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

import rospy
import rospkg
from tf_object_detection.srv import *

if tf.__version__ < '1.4.0':
	raise ImportError('Please upgrade your tensorflow installation to v1.4.* or later!')

from utils import label_map_util
from utils import visualization_utils as vis_util

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# TODO: Only download the model if it hasn't already been downloaded
def download_model(model_url, model_file):
	opener = urllib.request.URLopener()
	print "Downloading model..."
	opener.retrieve(model_url, model_file)
	print "Download complete!"
	tar_file = tarfile.open(model_file)
	for file in tar_file.getmembers():
		file_name = os.path.basename(file.name)
		if 'frozen_inference_graph.pb' in file_name:
			tar_file.extract(file, os.getcwd())

def load_model(ckpt_path):
	detection_graph = tf.Graph()
	with detection_graph.as_default():
		od_graph_def = tf.GraphDef()
		with tf.gfile.GFile(ckpt_path, 'rb') as fid:
			serialized_graph = fid.read()
			od_graph_def.ParseFromString(serialized_graph)
			tf.import_graph_def(od_graph_def, name='')
	return detection_graph

def load_label_map(labels_path, num_classes):
	label_map = label_map_util.load_labelmap(labels_path)
	categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes, use_display_name=True)
	category_index = label_map_util.create_category_index(categories)
	return category_index

class DetectedObject:

	def __init__(self, score, obj_class, left, top, right, bottom):
		self.score = score
		self.obj_class = obj_class
		self.left = left
		self.top = top
		self.right = right
		self.bottom = bottom

	def __str__(self):
		return "Class: %s\nScore: %s\nLeft: %s\nTop: %s\nRight: %s\nBottom: %s" % (self.obj_class, self.score, self.left, self.top, self.right, self.bottom)

def detect(image_np, detection_graph, category_index, tf_session):
	min_score_threshold = 0.5

	with detection_graph.as_default():
		# Definite input and output Tensors for detection_graph
		image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
		# Each box represents a part of the image where a particular object was detected.
		detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
		# Each score represent how level of confidence for each of the objects.
		# Score is shown on the result image, together with the class label.
		detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
		detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
		num_detections = detection_graph.get_tensor_by_name('num_detections:0')
		# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
		image_np_expanded = np.expand_dims(image_np, axis=0)
		# Actual detection.
		(boxes, scores, classes, num) = tf_session.run([detection_boxes, detection_scores, detection_classes, num_detections],feed_dict={image_tensor: image_np_expanded})
		# Visualization of the results of a detection.
		vis_util.visualize_boxes_and_labels_on_image_array(image_np,
		                                                   np.squeeze(boxes),
		                                                   np.squeeze(classes).astype(np.int32),
		                                                   np.squeeze(scores),
		                                                   category_index,
		                                                   use_normalized_coordinates=True,
		                                                   line_thickness=8,
		                                                   min_score_thresh=min_score_threshold)

		# Create a list of detected objects
		height, width, _ = image_np.shape
		detected_objects_list = []
		for i in range(0, num):
			# Only add to the list of detected objects if it is higher than the
			# threshold
			score = scores[0][i]
			if score >= min_score_threshold:
				obj_class = category_index[classes[0][i]]['name']
				top, left, bottom, right = boxes[0][i]
				# Multiply by the image dimensions as object coordinates are
				# normalized
				left *= width
				top *= height
				right *= width
				bottom *= height
				detected_object = DetectedObject(score, obj_class, left, top, right, bottom)
				detected_objects_list.append(detected_object)

	return (image_np, detected_objects_list)

def setup_object_detection():
	rospack = rospkg.RosPack()
	ros_package_path = rospack.get_path('tf_object_detection')

	# What model to download.
	MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
	MODEL_FILE = MODEL_NAME + '.tar.gz'
	DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

	# Path to frozen detection graph. This is the actual model that is used for
	# the object detection.
	PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

	# Get the absolute path to the object_detection data directory
	rospack = rospkg.RosPack()
	DATA_PATH = ros_package_path + '/src/object_detection/data'

	# List of the strings that is used to add correct label for each box.
	PATH_TO_LABELS = os.path.join(DATA_PATH, 'mscoco_label_map.pbtxt')

	NUM_CLASSES = 90

	model_url = DOWNLOAD_BASE + MODEL_FILE
	download_model(model_url, MODEL_FILE)

	detection_graph = load_model(PATH_TO_CKPT)
	category_index = load_label_map(PATH_TO_LABELS, NUM_CLASSES)

	return (detection_graph, category_index)

detection_graph = None
category_index = None
tf_session = None

def handle_object_detection(req):
	bridge = CvBridge()

	# Convert the ROS image message to a cv2 image
	try:
		cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")
	except CvBridgeError as e:
		print e

	# Since cv2 images are numpy NDarrays, it can simply be passed as one
	(cv_image, detected_object_list) = detect(cv_image, detection_graph, category_index, tf_session);

	# Convert the image with detected objects back into a ROS message
	result_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
	return ObjectDetectionResponse(result_msg)

def object_detection_server():
	global detection_graph, category_index, tf_session

	rospy.init_node('object_detection_server')

	(detection_graph, category_index) = setup_object_detection()
	tf_session = tf.Session(graph=detection_graph)
	service = rospy.Service('object_detection', ObjectDetection, handle_object_detection)
	print "Ready for object detection."

	rospy.spin()

	cv2.destroyAllWindows()

if __name__ == "__main__":
	object_detection_server()
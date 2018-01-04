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

def download_model(model_url, model_file):
	opener = urllib.request.URLopener()
	opener.retrieve(model_url, model_file)
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

def load_image_into_numpy_array(image):
	(im_width, im_height) = image.size
	return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

def detect(img_paths_list, detection_graph, category_index):
	# Size, in inches, of the output images.
	IMAGE_SIZE = (12, 8)

	with detection_graph.as_default():
		with tf.Session(graph=detection_graph) as sess:
			# Definite input and output Tensors for detection_graph
			image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
			# Each box represents a part of the image where a particular object was detected.
			detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
			# Each score represent how level of confidence for each of the objects.
			# Score is shown on the result image, together with the class label.
			detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
			detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
			num_detections = detection_graph.get_tensor_by_name('num_detections:0')
			for image_path in img_paths_list:
				image = Image.open(image_path)
				# the array based representation of the image will be used later in order to prepare the
				# result image with boxes and labels on it.
				image_np = load_image_into_numpy_array(image)
				# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
				image_np_expanded = np.expand_dims(image_np, axis=0)
				# Actual detection.
				(boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections],feed_dict={image_tensor: image_np_expanded})
				# Visualization of the results of a detection.
				vis_util.visualize_boxes_and_labels_on_image_array(image_np,
				                                                   np.squeeze(boxes),
				                                                   np.squeeze(classes).astype(np.int32),
				                                                   np.squeeze(scores),
				                                                   category_index,
				                                                   use_normalized_coordinates=True,
				                                                   line_thickness=8)
				plt.figure(figsize=IMAGE_SIZE)
				plt.imshow(image_np)
				plt.show()

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

	# For the sake of simplicity we will use only 2 images:
	# image1.jpg
	# image2.jpg
	# If you want to test the code with your images, just add path to the images to the TEST_IMAGE_PATHS.
	PATH_TO_TEST_IMAGES_DIR = ros_package_path + '/src/object_detection/test_images'
	TEST_IMAGE_PATHS = [os.path.join(PATH_TO_TEST_IMAGES_DIR, 'image{}.jpg'.format(i)) for i in range(1, 3) ]
	detect(TEST_IMAGE_PATHS, detection_graph, category_index)

def handle_object_detection(req):
	print "Not implemented!"

def object_detection_server():
	rospy.init_node('object_detection_server')

	setup_object_detection()
	service = rospy.Service('object_detection', ObjectDetection, handle_object_detection)
	print "Ready for object detection."

	rospy.spin()

if __name__ == "__main__":
	object_detection_server()
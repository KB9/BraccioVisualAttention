#!/usr/bin/env python

import sys
import requests
import json
import rospy
from std_msgs.msg import Int32MultiArray, Bool

payload = {}
headers = {'content-type': 'application/json'}
ip = 'http://192.168.0.71:4000/jsonrpc'
publisher = None

def onAnglesReceived(angles):
	global payload
	print "received angles"

	intValues = []

	# Ensure that all 3 axis of the point are provided, else keep Braccio steady
	if len(angles.data) == 5:
		try:
			for value in angles.data:
				intValues.append(int(float(value)))
		except ValueError as e:
			print "Integer values expected for the gaze point angles"
			print e
	else:
		print "Expecting 5 values for the gaze point angles (base, shoulder, elbow, wrist, wrist rotation), " + str(len(angles.data)) + " received. Command ignored."

	if len(intValues) > 0:
		payload = {
			"method": "set_braccio_gaze",
			"params": {"M1": intValues[0], "M2": intValues[1], "M3": intValues[2], "M4": intValues[3], "M5": intValues[4]},
			"jsonrpc": "2.0",
			"id": 0,
		}

	try:
		response = requests.post(ip, data=json.dumps(payload), headers=headers).json()
		print "Arduino Yun (" + ip + ") response:", response
		onGazeFocused()
	except requests.ConnectionError:
		print "Connection error. Is braccio_gaze_server.py running?"

def onGazeFocused():
	publisher.publish(True)

def main():
	global publisher
	
	rospy.init_node("braccio_gaze_controller", anonymous=False)
	publisher = rospy.Publisher("braccio_gaze_focus_callback", Bool, queue_size=10)
	subscriber = rospy.Subscriber("/braccio_gaze_focus_setter", Int32MultiArray, onAnglesReceived)
	rospy.spin()

if __name__ == "__main__":
	main()
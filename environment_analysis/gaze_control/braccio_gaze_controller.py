import sys
import requests
import json

payload = {}
headers = {'content-type': 'application/json'}

def main(angles, ip):
	global payload

	intValues = []

	# Ensure that all 3 axis of the point are provided, else keep Braccio steady
	if len(angles) == 5:
		try:
			for value in angles:
				intValues.append(int(float(value)))
		except ValueError as e:
			print "Integer values expected for the gaze point angles"
			print e
	else:
		print "Expecting 5 values for the gaze point angles (base, shoulder, elbow, wrist, wrist rotation), " + str(len(angles)) + " received. Command ignored."

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
	except requests.ConnectionError:
		print "Connection error. Is braccio_gaze_server.py running?"


if __name__ == "__main__":
	main(sys.argv[1:], 'http://192.168.0.71:4000/jsonrpc')
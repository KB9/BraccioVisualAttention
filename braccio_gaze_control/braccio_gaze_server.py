from werkzeug.wrappers import Request, Response
from werkzeug.serving import run_simple
from jsonrpc import JSONRPCResponseManager, dispatcher
import sys
sys.path.insert(0, '/usr/lib/python2.7/bridge')
from time import sleep
from bridgeclient import BridgeClient as bridgeclient
import csv

BRACCIO_IP = '192.168.0.71'
BRACCIO_PORT = 4000

value = bridgeclient()
gaze_point_angles = []

@dispatcher.add_method
def set_braccio_gaze(**kwargs):
	# Read in the new gaze point
	global gaze_points
	gaze_point_angles.append([str(kwargs["M1"]), str(kwargs["M2"]), str(kwargs["M3"]), str(kwargs["M4"]), str(kwargs["M5"])])

	executed = False
	value.put('RESULT', "incomplete")

	# Write the new gaze point to the CSV file
	with open("/mnt/sda1/gaze_point_angles.csv", "wb") as csv_file:
		writer = csv.writer(csv_file, delimiter=',')
		for angle in gaze_point_angles:
			writer.writerow((angle[0], angle[1], angle[2], angle[3], angle[4]))

	# Clear all gaze points
	reset_gaze_points()

	# Inform the movement controller that a new gaze point has been selected,
	# and wait for the completion of the movement
	value.put('new_gaze_point', 'P')
	while (not executed):
		result = value.get('RESULT')
		if result == "complete":
			executed = True

	# Set the response
	return "Focused on new gaze point"

def reset_gaze_points():
	global gaze_point_angles
	gaze_point_angles = []
	return

@Request.application
def application(request):
	# Dispatcher is dictionary {<method_name>: callable}
	response = JSONRPCResponseManager.handle(request.data, dispatcher)
	return Response(response.json, mimetype='application/json')

if __name__ == '__main__':
	run_simple(BRACCIO_IP, BRACCIO_PORT, application)
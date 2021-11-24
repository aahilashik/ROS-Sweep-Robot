"""
#!/usr/bin/env python

import rospy
import yaml
import cv2
import tf
import pickle
import actionlib
import numpy as np
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from coverage_path_planner import planning as CCPP # Complete coverage path planning (CCPP)
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def map2Img(x, y, wrtOrigin=True):
	global height
	if wrtOrigin: return int(x/resolution + originX), int(originY - y/resolution)
	else: return int(-x/resolution), int(height + y/resolution)

def img2Map(x, y):
	return int((x - originX) * resolution), int(-(y - originY) * resolution)

recorded = []
def callback_image(event, x, y, flags, param):
	global mode, recorded
	if event == cv2.EVENT_LBUTTONDOWN and mode == "record":
		recorded.append([x, y])
		print("Coordinates of pixel: X: ",x," Y: ",y)
	# elif event == cv2.EVENT_RBUTTONDOWN: print("Resetted : Low and High Range")

actual_pose = None
def callback_pose(msg):
	global actual_pose
	quaternion = (	msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w	)
	euler = tf.transformations.euler_from_quaternion(quaternion)

	# print("Pose : {} {}".format(msg.pose.pose.position.x, msg.pose.pose.position.y))
	x, y = map2Img(msg.pose.pose.position.x, msg.pose.pose.position.y)
	actual_pose = x, y, euler[2] 
	# print ("Odom : ", actual_pose)

# def callback_map(data):
#	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#	print(type(data.data), len(data.data))

trajectory = None
def callback_path(data):
	global trajectory
	trajectory = np.array([[list(map2Img(pose.pose.position.x, pose.pose.position.y)) for pose in data.poses]])
	# print("Trajectory Updated")
	
mbClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
goal = MoveBaseGoal()
def movebaseController(sweepArea):
	mbClient.wait_for_server()

	# Navigate through Sweep Path
	targetPoints = division[sweepArea][1][0]
	for x, y in targetPoints:
		x, y = img2Map(x, y)
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.w = 1.0

		mbClient.send_goal(goal)
		print("Destination - X : {} | Y : {}".format(x, y))
		
		cmd = None
		while True:
			cmd = updateMap(sweepArea)
			if cmd == "q": break

			if   mbClient.get_state() == 1: continue
			elif mbClient.get_state() == 3: break
			else : 
				print("MB Stopped! State : ", mbClient.get_state())
				break
		if cmd == "q": break
		if mbClient.get_result(): print("Reached - X : {} | Y : {}".format(x, y))
	
resCCPP = 2.5 # Tune Parameter for CCPP
mapCache = {}
def updateMap(sweepArea=None, pts=None):
	global actual_pose, mode, recorded, division, trajectory
	mapImg = mapImgOrig.copy()

	# Fill Recorded Division in Map
	if len(division.values()) > 0:
		divArea = np.zeros_like(mapImg, np.uint8)
		for dName, dPoints, color in division.values():
			divCenterX, divCenterY = dPoints.mean(axis=1).flatten()
			textSize = cv2.getTextSize(dName, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)[0]
			textX, textY = int(divCenterX - textSize[0]/2), int(divCenterY + textSize[1]/2)
			mapImg = cv2.putText(mapImg, dName, (textX, textY), cv2.FONT_HERSHEY_DUPLEX, 0.5, color, 1)
			divArea = cv2.drawContours(divArea, dPoints, -1, color=color, thickness=cv2.FILLED)

		mask = divArea.astype(bool)
		mapImg[mask] = cv2.addWeighted(mapImg, 0.5, divArea, 0.5, 0)[mask]

		# Plot Coverage Planner Path
		if sweepArea != None:
			if sweepArea in mapCache: 
				path = mapCache[sweepArea]
			else:
				cPath = division[sweepArea][1]
				pointS = np.concatenate((cPath[0], cPath[0][0:1]), axis=0) * 1
				path = CCPP(pointS[:, 0], pointS[:, 1], resCCPP)
				mapCache[sweepArea] = path
			pointS = np.concatenate(([path[0]], [path[1]]), axis=0).T
			mapImg = cv2.polylines(mapImg, (np.array([pointS])/1).astype(int), False, (25, 64, 15), 1)
		
	# Plot Robot Pose
	if actual_pose!= None:
		X, Y, theta = actual_pose
		mapImg = cv2.circle(mapImg, (X, Y), 4, (15, 0, 63), -1)
		mapImg = cv2.arrowedLine(mapImg, (X, Y), (X + int(8 * np.cos(theta)), Y - int(8 * np.sin(theta))), (15, 0, 63), 1, 8, 0, 0.6)

	# Plot Trajectory Path
	if trajectory!= None and sweepArea != None:
		mapImg = cv2.putText(mapImg, "In motion", (0, mapImg.shape[0]-10), cv2.FONT_HERSHEY_DUPLEX, 2.0, (30, 200, 15), 2)
		mapImg = cv2.polylines(mapImg, trajectory, False, (30, 200, 15), 1)
	
	# Plot Recording Division in Map
	if mode == "record": 
		mapImg = cv2.putText(mapImg, "Record : "+divName, (0, mapImg.shape[0]-10), cv2.FONT_HERSHEY_DUPLEX, 2.0, (0, 64, 15), 2)
		mapImg = cv2.polylines(mapImg, np.array([recorded]), True, (0, 64, 15), 2)

	#############
	try:
		a,b = pts
		mapImg = cv2.line(mapImg, (a-10, b), (a+10, b), (0, 128, 255), 2)
		mapImg = cv2.line(mapImg, (a, b-10), (a, b+10), (0, 128, 255), 2)
	except: pass
	#############


	cv2.imshow("RealTime Map", mapImg)

	try: 	return chr(cv2.waitKey(1))
	except: return False


    
mode = "normal"
def main():
	global mode, recorded, division
	
	rospy.init_node('map_builder', anonymous=True)

	# rospy.Subscriber("/map", OccupancyGrid, callback_map)
	rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback_pose)  
	rospy.Subscriber("/move_base/TrajectoryPlannerROS/global_plan", Path, callback_path)
	
	movebaseController("Bedroom") ###########
	# go_to_goal(535, 285)
	while True:
		cmd = updateMap()
		if cmd: print("CMD : ", cmd)
		if   cmd == "r":
			mode = "record"
			divName = str(raw_input("Enter Division Name : "))
		elif cmd == "n": mode = "normal"
		elif cmd == "s" and mode == "record":
			division[divName] = [divName, np.array([recorded]), tuple(np.random.randint(256, size=3))]
			recorded = []
			mode = "normal"
		elif (cmd in ['\x08', '\xff']) and len(recorded)>0: recorded.pop()
		elif cmd == "q": break
	cv2.destroyAllWindows()
	
	# Store Updated Division Data
	with open('/home/ubunturog/catkin_ws/src/udacity_bot/config/division.p', 'wb') as handle:
		pickle.dump(division, handle, protocol=pickle.HIGHEST_PROTOCOL)

def go_to_goal(x, y):
	mbClient.wait_for_server()
	goal.target_pose.header.frame_id = "map"

	X, Y = x, y
	x, y = img2Map(x, y)
	print("Destination - X : {} | Y : {}".format(x, y))
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.orientation.w = 1.0

	mbClient.send_goal(goal)
	
	while True:
		if updateMap(pts=(X,Y)) == "q": break

		if   mbClient.get_state() == 1: continue
		elif mbClient.get_state() == 3: break
		else: 
			print("MB Stopped! State : ", mbClient.get_state())
			break
	if mbClient.get_result(): print("Reached - X : {} | Y : {}".format(x, y))


scale = 0.25 # Image Scale

# File Path
yamlFilePath 	= "/home/ubunturog/catkin_ws/src/udacity_bot/maps/jackal_race.yaml"
mapImgFilePath 	= "/home/ubunturog/catkin_ws/src/udacity_bot/maps/jackal_race.pgm"
pickleFilePath 	= "/home/ubunturog/catkin_ws/src/udacity_bot/config/division.p"

cv2.namedWindow('RealTime Map')
cv2.setMouseCallback('RealTime Map', callback_image)

# Load Configuration Yaml
with open(yamlFilePath, "r") as stream:
	yamlConf = yaml.safe_load(stream)
	oX, oY, _ = yamlConf.get("origin")
	resolution = yamlConf.get("resolution")/scale

# Load Existing Division Data
try:
	with open(pickleFilePath, 'rb') as handle:
		division = pickle.load(handle)
except: division = {}

# Load Map Image, Scale It and Denote Origin
mapImgOrig = cv2.imread(mapImgFilePath) 
mapImgOrig = cv2.resize(mapImgOrig, (0, 0), fx=scale, fy=scale)
height, width, depth = mapImgOrig.shape
# originX, originY = int(-oX/resolution), int(height + oY/resolution)
originX, originY = map2Img(oX, oY, False)
mapImgOrig = cv2.line(mapImgOrig, (originX-10, originY), (originX+10, originY), (128, 0, 128), 1)
mapImgOrig = cv2.line(mapImgOrig, (originX, originY-10), (originX, originY+10), (128, 0, 128), 1)

		
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program Stopped!!")
"""

#!/usr/bin/env python

import rospy
import yaml
import cv2
import tf
import time
import pickle
import actionlib
import numpy as np
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from coverage_path_planner import planning as CCPP # Complete coverage path planning (CCPP)
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tqdm import tqdm


def map2Img(x, y, wrtOrigin=True):
	global height
	if wrtOrigin: return int(x/resolution + originX), int(originY - y/resolution)
	else: return int(-x/resolution), int(height + y/resolution)

def img2Map(x, y):
	return float((x - originX) * resolution), float((originY - y) * resolution)

recorded = []
def callback_image(event, x, y, flags, param):
	global mode, recorded
	if event == cv2.EVENT_LBUTTONDOWN and mode == "record":
		recorded.append([x, y])
		print("Coordinates of pixel: X: ",x," Y: ",y)
	# elif event == cv2.EVENT_RBUTTONDOWN: print("Resetted : Low and High Range")

actual_pose = None
def callback_pose(msg):
	global actual_pose
	quaternion = (	msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w	)
	euler = tf.transformations.euler_from_quaternion(quaternion)

	# print("Pose - X : {} | Y: {}".format(msg.pose.pose.position.x, msg.pose.pose.position.y))
	x, y = map2Img(msg.pose.pose.position.x, msg.pose.pose.position.y)
	actual_pose = x, y, euler[2]
	# print ("{} - Odom - X : {} | Y: {}".format(time.strftime("%H:%M:%S"), x, y))

# def callback_map(data):
#	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#	print(type(data.data), len(data.data))

trajectory = None
def callback_path(data):
	global trajectory
	trajectory = np.array([[list(map2Img(pose.pose.position.x, pose.pose.position.y)) for pose in data.poses]])
	# print("Trajectory Updated")
	
mbClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
goal = MoveBaseGoal()
def movebaseController(sweepArea):
	def findOrientation(pt1, pt2):
		pt1, pt2 = img2Map(pt1[0], pt1[1]), img2Map(pt2[0], pt2[1])
		yaw = np.arctan2(pt2[1]-pt1[1], pt2[0]-pt1[0])

		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		return quaternion[0], quaternion[1], quaternion[2], quaternion[3]		

	mbClient.wait_for_server()

	# Navigate through Sweep Path
	generateCoveragePath(sweepArea)		#######
	targetPoints = mapCache[sweepArea][0]
	# print("Count - ", len(targetPoints))
	for (X, Y), (X1, Y1) in tqdm(zip(targetPoints[0:-1], targetPoints[1:])):
		x, y = img2Map(X, Y)
		ox, oy, oz, ow = findOrientation((X, Y), (X1, Y1))

		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()

		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.position.z = 0.0
		goal.target_pose.pose.orientation.x = ox
		goal.target_pose.pose.orientation.y = oy
		goal.target_pose.pose.orientation.z = oz
		goal.target_pose.pose.orientation.w = ow

		mbClient.send_goal(goal)
		# print("Remaining : {} / {} \nDestination - X : {} | Y : {}".format(11, 2, x, y))
		
		cmd = None
		while True:
			cmd = updateMap(sweepArea, pts=(X, Y))
			if cmd == "q": break

			if   mbClient.get_state() == 1: continue
			elif mbClient.get_state() == 3: print("MB Break"); break
			else : 
				print("MB Stopped! State : ", mbClient.get_state())
				break
		if cmd == "q": break
		if mbClient.get_result(): pass # print("Reached - X : {} | Y : {}".format(x, y))

		"""
		for i in range(4):
			updateMap(sweepArea, pts=(X,Y))		# No need
			rospy.sleep(1)			# No need
		"""


def generateCoveragePath(sweepArea):
	if not sweepArea in mapCache: 
		cPath = division[sweepArea][1]
		pointS = np.concatenate((cPath[0], cPath[0][0:1]), axis=0) * 1
		path = CCPP(pointS[:, 0], pointS[:, 1], resCCPP)
		pointS = np.concatenate(([path[0]], [path[1]]), axis=0).T
		mapCache[sweepArea] = (np.array([pointS]) / 1).astype(int)
	return mapCache[sweepArea]


resCCPP = 3 # 4 # 2.5 # Tune Parameter for CCPP
mapCache = {}
def updateMap(sweepArea=None, pts=None):
	global actual_pose, mode, recorded, division, trajectory, divName
	mapImg = mapImgOrig.copy()

	# Fill Recorded Division in Map
	if len(division.values()) > 0:
		divArea = np.zeros_like(mapImg, np.uint8)
		for dName, dPoints, color in division.values():
			divCenterX, divCenterY = dPoints.mean(axis=1).flatten()
			textSize = cv2.getTextSize(dName, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)[0]
			textX, textY = int(divCenterX - textSize[0]/2), int(divCenterY + textSize[1]/2)
			mapImg = cv2.putText(mapImg, dName, (textX, textY), cv2.FONT_HERSHEY_DUPLEX, 0.5, color, 1)
			divArea = cv2.drawContours(divArea, dPoints, -1, color=color, thickness=cv2.FILLED)

		mask = divArea.astype(bool)
		mapImg[mask] = cv2.addWeighted(mapImg, 0.5, divArea, 0.5, 0)[mask]

		# Plot Coverage Planner Path
		if sweepArea != None:
			path = generateCoveragePath(sweepArea)
			mapImg = cv2.polylines(mapImg, path, False, (25, 64, 15), 1)


		
	# Plot Robot Pose
	if actual_pose!= None:
		X, Y, theta = actual_pose
		mapImg = cv2.circle(mapImg, (X, Y), 4, (15, 0, 63), -1)
		mapImg = cv2.arrowedLine(mapImg, (X, Y), (X + int(8 * np.cos(theta)), Y - int(8 * np.sin(theta))), (15, 0, 63), 1, 8, 0, 0.6)

	# Plot Trajectory Path
	if type(trajectory)!= type(None): # and sweepArea != None:
		mapImg = cv2.putText(mapImg, "In motion", (0, mapImg.shape[0]-10), cv2.FONT_HERSHEY_DUPLEX, 2.0, (30, 200, 15), 2)
		mapImg = cv2.polylines(mapImg, trajectory, False, (30, 200, 15), 1)
	
	# Plot Recording Division in Map
	if mode == "record": 
		mapImg = cv2.putText(mapImg, "Record : "+divName, (0, mapImg.shape[0]-10), cv2.FONT_HERSHEY_DUPLEX, 2.0, (0, 64, 15), 2)
		mapImg = cv2.polylines(mapImg, np.array([recorded]), True, (0, 64, 15), 2)

	#############
	try:
		a,b = pts
		mapImg = cv2.line(mapImg, (a-5, b), (a+5, b), (0, 128, 255), 2)
		mapImg = cv2.line(mapImg, (a, b-5), (a, b+5), (0, 128, 255), 2)
	except: pass
	#############


	cv2.imshow("RealTime Map", mapImg)

	try: 	return chr(cv2.waitKey(1))
	except: return False


    
mode = "normal"
divName = ""
def main():
	global mode, recorded, division, divName
	
	rospy.init_node('map_builder', anonymous=True)

	# rospy.Subscriber("/map", OccupancyGrid, callback_map)
	rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback_pose)  
	rospy.Subscriber("/move_base/TrajectoryPlannerROS/global_plan", Path, callback_path)
	
	print("Choose the Room to Clean : ")
	print("\n".join(["{} : {}".format(idx+1, room) for idx, room in enumerate(division.keys())]))
	print("Or Enter \"0\" to see the map  ")
	
	idx = int(input("Enter the option here >>> "))
	if idx >= 1: movebaseController(division.keys()[idx-1])
	else:
		print("Press \'r\' key to record new area. \nPress \'c\' to clean the area. \nPress \'q\' to quit.")

	# movebaseController("Bedroom")			###########
	# go2Point(570, 170) 				###########
	# go2Point(525, 280) 				###########
	while True:
		cmd = updateMap()
		# if cmd: print("CMD : {}".format(cmd))
		if   cmd == "r":
			mode = "record"
			divName = str(raw_input("Enter Division Name : "))
		elif cmd == "c": 
			print("Choose the Room to Clean : ")
			print("\n".join(["{} : {}".format(idx+1, room) for idx, room in enumerate(division.keys())]))
			print("Or Enter \"0\" to see the map  ")
			
			idx = int(input("Enter the option here >>> "))
			if idx >= 1: movebaseController(division.keys()[idx-1])
			else: 
				print("Press \'r\' key to record new area. \nPress \'c\' to clean the area. \nPress \'q\' to quit.")
		elif cmd == "n": mode = "normal"
		elif cmd == "s" and mode == "record":
			division[divName] = [divName, np.array([recorded]), tuple(np.random.randint(255, size=3))]
			recorded = []
			mode = "normal"
		elif (cmd in ['-', '-']) and len(recorded)>0: recorded.pop()
		elif cmd == "q": break
	cv2.destroyAllWindows()
	
	# Store Updated Division Data
	with open(pickleFilePath, 'wb') as handle:
		pickle.dump(division, handle, protocol=pickle.HIGHEST_PROTOCOL)

def go2Point(x, y):
	mbClient.wait_for_server()
	goal.target_pose.header.frame_id = "map"

	X, Y = x, y
	x, y = img2Map(x, y)
	print("Destination - X : {} | Y : {}".format(x, y))
	goal.target_pose.header.stamp = rospy.Time.now()
	
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = 0.0
	goal.target_pose.pose.orientation.x = 0.0
	goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = 0.0
	goal.target_pose.pose.orientation.w = 1.0

	mbClient.send_goal(goal)
	
	while True:
		if updateMap(pts=(X,Y)) == "q": break

		if   mbClient.get_state() == 1: continue
		elif mbClient.get_state() == 3: break
		else: 
			print("MB Stopped! State : ", mbClient.get_state())
			break
	if mbClient.get_result(): print("Reached - X : {} | Y : {}".format(x, y))


scale = 0.25 # Image Scale

# File Path
yamlFilePath 	= "/home/ubuntussd/catkin_ws/src/sweep_robot/maps/jackal_race.yaml"
mapImgFilePath 	= "/home/ubuntussd/catkin_ws/src/sweep_robot/maps/jackal_race.pgm"
pickleFilePath 	= "/home/ubuntussd/catkin_ws/src/sweep_robot/config/division.p"

cv2.namedWindow('RealTime Map')
cv2.setMouseCallback('RealTime Map', callback_image)

# Load Configuration Yaml
with open(yamlFilePath, "r") as stream:
	yamlConf = yaml.safe_load(stream)
	oX, oY, _ = yamlConf.get("origin")
	resolution = yamlConf.get("resolution")/scale

# Load Existing Division Data
try:
	with open(pickleFilePath, 'rb') as handle:
		division = pickle.load(handle)
except: division = {}

# Load Map Image, Scale It and Denote Origin
mapImgOrig = cv2.imread(mapImgFilePath) 
mapImgOrig = cv2.resize(mapImgOrig, (0, 0), fx=scale, fy=scale)
height, width, depth = mapImgOrig.shape
# originX, originY = int(-oX/resolution), int(height + oY/resolution)
originX, originY = map2Img(oX, oY, False)
mapImgOrig = cv2.line(mapImgOrig, (originX-10, originY), (originX+10, originY), (128, 0, 128), 1)
mapImgOrig = cv2.line(mapImgOrig, (originX, originY-10), (originX, originY+10), (128, 0, 128), 1)


# division.pop("world")
division.pop("Hall")		
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program Stopped!!")

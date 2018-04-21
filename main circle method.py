from robot import Robot, WALL, TOKEN_ZONE_0, TOKEN_ZONE_1, TOKEN_ZONE_2, TOKEN_ZONE_3, COLUMN, PinMode, PinValue, GameMode
import time
import threading
from math import *


MIN_LOOK_ANGLE = -70
MAX_LOOK_ANGLE = 70

global robotFacing



motorCfg = {
	0: {  # FR
		'gpio': 4,
		'servo': 0,
		'invert': 0,
		'currentSpeed': 0,
		'extra': -0.0
	},
	1: {  # FL
		'gpio': 5,
		'servo': 1,
		'invert': 0,
		'currentSpeed': 0,
		'extra': -0.0
	},
	2: {  # BL
		'gpio': 6,
		'servo': 2,
		'invert': 0,
		'currentSpeed': 0,
		'extra': 0
	},
	3: {  # BR
		'gpio': 7,
		'servo': 3,
		'invert': 0,
		'currentSpeed': 0,
		'extra': 0
	}
}


def changeMotorSpeed(velocityArray):  # FR, FL, BL, BR
	#print("Velocity Array")
	#print(velocityArray)
	robot.stopped = True
	motorNumber = 0
	motorSpeeds = [0, 0, 0, 0]
	for speed in velocityArray:
		invert = False

		motorInfo = motorCfg[motorNumber]

		if speed is None:
			speed = motorCfg[motorNumber]['currentSpeed']


		motorCfg[motorNumber]['currentSpeed'] = speed

		if speed < 0:
			invert = not invert
			speed = -speed

		if speed > 0:
			robot.stopped = False

		if motorInfo['invert'] == 1:
			invert = not invert

		if invert is True:
			r.servo_board.gpios[motorInfo['gpio']].mode = PinMode.OUTPUT_LOW
		else:
			r.servo_board.gpios[motorInfo['gpio']].mode = PinMode.OUTPUT_HIGH

		motorSpeeds[motorNumber] = round(speed)

		motorNumber += 1

	#print("Motor Speeds before:")
	#print(motorSpeeds)
	motorSpeeds = mapToPWM(motorSpeeds)
	#print("Motor Speeds after:")
	#print(motorSpeeds)
	r.servo_board.direct_command('m', motorSpeeds[3],3)
	#print(motorSpeeds[0])
	r.servo_board.direct_command('m', motorSpeeds[2],2)
	#print(motorSpeeds[1])
	r.servo_board.direct_command('m', motorSpeeds[1],1)
	#print(motorSpeeds[2])
	r.servo_board.direct_command('m', motorSpeeds[0],0)
	#print(motorSpeeds[3])
	#r.servo_board.direct_command('m', 1,4)
# 	m is just a function that takes 4 ints between 0 and 100

def mapToPWM(motorSpeeds):
	for i in range(len(motorSpeeds)):
		motorSpeeds[i] = round(mapThing(motorSpeeds[i],0,100,0,4096))
	return motorSpeeds
	

class OurRobot:

	def __init__(self):
		self.position = Vector(4, 4)
		self.cameraAngle = 0
		self.facing = 0
		self.armRaised = True
		self.stopped = True
		self.currentArmAngle = -90



class Vector:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def __add__(self, other):  # both must be Vector
		newX = self.x + other.x
		newY = self.y + other.y
		return Vector(newX, newY)

	def __sub__(self, other):  # both must be Vector
		newX = self.x - other.x
		newY = self.y - other.y
		return Vector(newX, newY)

	def __mul__(self, other):
		if type(other).__name__ == "Vector":
			newX = self.x * other.x
			newY = self.y * other.y
			return Vector(newX, newY)
		else:
			newX = self.x * other
			newY = self.y * other
			return Vector(newX, newY)

	def __rmul__(self, other):
		return self.__mul__(other)

	def __neg__(self):
		return Vector(-self.x, -self.y)

	def __truediv__(self, other):
		if type(other).__name__ == "Vector":
			newX = self.x / other.x
			newY = self.y / other.y
			return Vector(newX, newY)
		else:
			newX = self.x / other
			newY = self.y / other
			return Vector(newX, newY)

	def __div__(self, other):
		if type(other).__name__ == "Vector":
			newX = self.x / other.x
			newY = self.y / other.y
			return Vector(newX, newY)
		else:
			newX = self.x / other
			newY = self.y / other
			return Vector(newX, newY)

	def __str__(self):
		return "V(" + str(self.x) + ", " + str(self.y) + ")"

	def __eq__(self, other):
		return self.x == other.x and self.y == other.y

	def magnitude(self):
		return (self.x ** 2 + self.y ** 2) ** 0.5

	def normalize(self):
		magnitude = self.magnitude()
		if magnitude > 0:
			self.x = self.x / magnitude
			self.y = self.y / magnitude

		return self

	def unitVector(self):
		if self.magnitude() > 0:
			return self.__mul__(1/self.magnitude())
		else:
			return Vector(0, 0)

	def xVal(self):
		return self.x

	def yVal(self):
		return self.y

V = Vector

# positions are in meters
class Marker:
	def __init__(self, pos, markerType, markerId, norm=0):
		self.position = pos
		self.markerType = markerType
		# 0deg/rads is to the top, I think
		self.normal = norm
		self.normalDeg = 180 * norm / pi
		self.markerId = markerId
		self.lastSeen = 0
		self.lastNormalUpdate = 0
		self.lastNormalAngle = 0
		self.grabbed = False


	def grab(self):
		print("Grabbing: "+ self.markerId + " at "+ self.position)
		turnToFaceVec(30, self.position)

		# Might move to about 0.4m from marker
		moveTo((self.position - robotPosition).unitVector() * ((self.position - robotPosition).magnitude() - 0.4))

		raiseArm()
		# Move by 0.1m with arm up
		moveByLocalVec(V(0.0707, 0.0707))

		if lowerArm():
			return True
		else:
			# If arm doesn't lower
			pass


# Arm stuff

def changeArmAngle(angle):
	if abs(angle - robot.currentArmAngle) > 0:
		print(robot.currentArmAngle)
		print(angle)
		print(sign(angle - robot.currentArmAngle))
		for i in range(robot.currentArmAngle, angle, sign(angle - robot.currentArmAngle)*15):
			print("i = " + str(i))
			# turnServo(5, -90 - i)
			# turnServo(6, i)
			robot.currentArmAngle = i
			time.sleep(0)
		turnServo(5, -90 - angle)
		turnServo(6, angle)


def raiseArm():
	# TODO Change angle maybe
	if robot.stopped:
		changeArmAngle(-89)
		robot.armRaised = True
		return True
	else:
		return False

def checkIfArmIsStuck():
	if r.servo_board.gpios[2].read() == PinValue.HIGH:
		return True
	else:
		return False

def lowerArm():
	if robot.armRaised:
		targetAngle = -40

		changeArmAngle(targetAngle)



		time.sleep(0.4)
		if checkIfArmIsStuck():
			raiseArm()
			return False
		robot.armRaised = False
		return True



def lockedSleep(t):  # Use instead of time.sleep() in competition
	robotLock.acquire()
	time.sleep(t)
	robotLock.release()


def stop():
	changeMotorSpeed([0, 0, 0, 0])


# Return sign of number
def sign(num):
	if num == 0:
		return 0
	return 1 if (num > 0) else -1


# Movement

def move(direction, speed, time, degrees=True, autoStop=True):  # Magic
	if degrees:
		direction = (pi * direction / 180)

	#direction = 2*pi - (direction % (2 * pi))

	mflPower = cos(direction + pi * 0.25)
	mbrPower = cos(direction + pi * 0.25)
	mfrPower = sin(direction + pi * 0.25)
	mblPower = sin(direction + pi * 0.25)

	normalisationFactor = max(abs(mflPower), abs(mfrPower), abs(mblPower), abs(mbrPower))

	motor0 = (speed * mfrPower / normalisationFactor) - 1 * sign(mfrPower)
	motor1 = (speed * mflPower / normalisationFactor) + 0 * sign(mflPower)
	motor2 = (speed * mblPower / normalisationFactor) - 1 * sign(mblPower)
	motor3 = (speed * mbrPower / normalisationFactor) + 0 * sign(mbrPower)
	
	velocityArray = [motor0,motor1,motor2,motor3]
	
	changeMotorSpeed(velocityArray)


	lockedSleep(time)

	if autoStop:
		stop()


def turn(speed, time, autoStop=True):
	changeMotorSpeed([-speed, speed, speed, -speed])

	lockedSleep(time)
	if autoStop:
		stop()


def turnByAngle(speed, angle, degrees=True, autoStop=True):  # Magic
	if not degrees:
		angle = 180 * angle / pi

	if angle > 180:
		angle = -(360 - angle)

	if angle < -180:
		angle = 360 + angle

	if abs(angle) > 1:
		turn((speed + 10) * sign(angle), abs(angle / speed) * 0.5, autoStop)


def turnToAngle(speed, angle, tolerance=30, updatePos=True, checking=True):  # Magic
	lockedSleep(0.5)
	if updatePos:
		updatePosition()

	print("LOOK ANGLE")
	print(angle)
	print("ROBOT LOOK ANGLE")
	print(robotFacing)

	ang = (angle % 360) - robotFacing

	if ang > 180:
		ang = -(360 - ang)

	if ang < -180:
		ang = 360 + ang

	print("WILL TURN")
	print(ang)
	if abs(ang) > tolerance:
		if checking:
			turnByAngle(speed, ang / 1.5)
			turnToAngle(speed, angle, tolerance)
		else:
			turnByAngle(speed, ang)


def turnToFaceVec(speed, pos, tolerance=30, update=True, checking=True):  # Magic
	if update:
		updatePosition()

	print("TURNING TO")
	print(pos)
	print("ROBOT POS")
	print(robotPosition)

	lookVec = pos - robotPosition
	lookAng = 180 * atan2(lookVec.x, lookVec.y) / pi
	turnToAngle(speed, lookAng, tolerance, False, checking)



def moveTo(pos, endAngle=False, forcePositionUpdate=True, tolerance=0.3):  # Magic
	updated = updatePosition()
	if forcePositionUpdate and not updated:
		while not updated:
			updated = updatePosition()
			stop()
			lockedSleep(0.01)

	print("CURRENT POSITION")
	print(robotPosition)
	print("GOING TO")
	print(pos)

	moveVec = pos - robotPosition
	moveDir = 180 * atan2(moveVec.x, moveVec.y) / pi  # use xy to flip axis (ang from y)
	moveDir -= robotFacing

	print("MOVE VEC")
	print(moveVec)
	print(moveVec.magnitude())
	print(tolerance)

	speed = max(20 * min(moveVec.magnitude(), 1), 20)

	if moveVec.magnitude() > tolerance:
		move(moveDir, speed, distanceToTime(speed, moveVec.magnitude() * 0.7), degrees=True, autoStop=True)
		lockedSleep(0.1)
		if endAngle:
			turnByAngle(40, (endAngle - robotFacing), degrees=True, autoStop=True)

		moveTo(pos, endAngle, forcePositionUpdate, tolerance)
	else:
		stop()





def moveByLocalVec(moveVec, maxSpeed=20, minSpeed=10):  # Magic
	moveDir = 180 * atan2(moveVec.x,  moveVec.y) / pi
	speed = max(maxSpeed * 2 * min(moveVec.magnitude(), 0.5), minSpeed)

	if moveVec.magnitude() > 0.05:
		move(moveDir, speed, distanceToTime(speed, moveVec.magnitude()), degrees=True, autoStop=True)
	else:
		stop()



def distanceToTime(speed, distance):
	if speed != 0:
		return distance / (2 * (speed / 100.0))
	else:
		return 0










# Markers

def getClosestMarker(markerType, updateMarkers=True, notGrabbed=True):
	if updateMarkers:
		updateTokenLocations()

	closest = None
	closestDist = 100
	for marker in markers:
		if marker.markerType == markerType and (not marker.grabbed if notGrabbed else True):
			distance = (marker.position - robotPosition).magnitude()
			if distance < closestDist:
				closest = marker
				closestDist = distance

	return closest

def updateTokenLocations(visionTable=False):  # Magic
	if not visionTable:
		visionTable = r.camera.see()
	for m in visionTable:
		if getMarkerType(m.id) in (TOKEN_ZONE_0, TOKEN_ZONE_1, TOKEN_ZONE_2, TOKEN_ZONE_3):
			angle = pi * (robotFacing + m.spherical.rot_y_degrees + robot.cameraAngle) / 180
			offset = m.spherical.distance_metres * Vector(sin(angle), cos(angle))

			position = robotPosition + offset

			print("FOUND MARKER AT")
			print(position)

			markers[m.id].lastSeen = time.time()
			markers[m.id].position = position


			markers[m.id].marketType = getMarkerType(m.id)


def getMarkerType(markerId):
	if markerId >= 0:
		if markerId <= 27:
			return WALL
		elif 27 < markerId <= 43:
			return COLUMN
		elif 44 <= markerId <= 48:
			return TOKEN_ZONE_0
		elif 49 <= markerId <= 53:
			return TOKEN_ZONE_1
		elif 54 <= markerId <= 58:
			return TOKEN_ZONE_2
		elif 59 <= markerId <= 63:
			return TOKEN_ZONE_3


def getMarkerOfType(markerType):
	# marker = None
	testCount = 0
	while testCount < 9:
		marker = getClosestMarker(markerType)
		if not marker:
			testCount += 1
			turnByAngle(40, 45)
			time.sleep(1)
		else:
			return marker

	moveTo(Vector(4, 4))
	updatePosition()



def updatePosition(force=True):  # Magic, but modified to make it easier to read and put all in one loop
	cameraAngleArray = [0, 0, 45, -45, -89, 89]
	mNumber = 0
	loopNumber = 0
	while mNumber <= 0 and loopNumber<6:
		turnCameraServo(cameraAngleArray[loopNumber])

		time.sleep(0.5)


		visionTable = sorted(r.camera.see(), key=lambda x: abs(x.spherical.distance_metres))

		mNumber = 0
		totalPos = Vector(0, 0)
		totalFacing = 0

		for m in visionTable:
			marker = markers[m.id]
			if marker != False and marker.markerType == (WALL or COLUMN):
				rotation = pi * m.spherical.rot_y_degrees / 180
				offset = Vector(sin(-rotation + marker.normal),
								cos(-rotation + marker.normal))

				# offset *= m.centre.polar.length
				offset *= m.spherical.distance_metres

				mNumber += 1
				totalPos += marker.position - offset


				global robotFacing
				robotFacing = (-m.spherical.rot_y_degrees - m.pixel_centre[1] + marker.normalDeg + -robot.cameraAngle) % 360

				robot.facing = robotFacing

				totalFacing += robotFacing


		updateTokenLocations(visionTable=visionTable)


		# lockedSleep(0.5)

		loopNumber += 1
		if mNumber > 0:
			global robotPosition
			robotPosition = totalPos / mNumber
			robot.position = totalPos / mNumber
			# turnCameraServo(0)
			return True

	turnCameraServo(0)
	if force:
		turnByAngle(40, 180)
		return updatePosition()




# Servos

def turnCameraServo(angle, degrees=True):
	if not degrees:
		angle = 180 * angle / pi

	# if angle < MIN_LOOK_ANGLE:
	# 	angle = MIN_LOOK_ANGLE
	# elif angle > MAX_LOOK_ANGLE:
	# 	angle = MAX_LOOK_ANGLE


	value = mapThing(angle, -90, 90, -0.61, 0.61)
	# value = mapThing(angle, -90, 90, -1, 1)

	robot.cameraAngle = angle

	turnServo(4, False, False, value)





def mapThing(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def turnServo(servoNum, angle, degrees=True, value=False):
	if servoNum > 3:
		if value == False and type(value) != int:
			if not degrees:
				angle = 180 * angle / pi

			value = mapThing(angle, -90, 90, -1, 1)

		print(value)
		r.servo_board.servos[servoNum].position = value
	else:
		print("0-3 are motors")
		Exception(IndexError)
		
		
def get_vision_table():
	visionTable = []
	for i in range(5):
		current_image = r.camera.see()
		for marker in current_image:
			if marker.id not in [m.id for m in visionTable]:
				visionTable.append(marker)
		print("MARKERS SEEN")
		print([m.id for m in visionTable])
	return visionTable


def find_token(arena = 0):
	if arena ==0:
		token_table = [44,45,46,47,48]
	elif arena == 1:
		token_table = [49,50,51,52,53]
	elif arena == 2:
		token_table = [54,55,56,57,58]
	elif arena == 3:
		token_table = [59,60,61,62,63]
	visionTable = get_vision_table()
	min_dist = 100
	dist = 200
	for marker in visionTable:
		if marker.id in token_table :
			dist = sqrt((marker.cartesian.x ** 2) + (marker.cartesian.z ** 2))
		if dist<min_dist:
			min_dist = dist
			target_token = visionTable.index(marker)
			target_x = marker.cartesian.x
			target_y = marker.cartesian.z
	if min_dist == 100:
		turnByAngle(15,10)
		find_token(arena)
		return False
	else:
		moveByLocalVec(V(target_x,target_y))
		changeArmAngle(-18)
		return True
	
def find_corner(arena = 0):
	if arena ==0:
		home_table = [0,1,2,3,25,26,27]
	elif arena == 1:
		home_table = [4,5,6,7,8,9]
	elif arena == 2:
		home_table = [11,12,13,14,15,16]
	elif arena == 3:
		home_table = [18,19,29,21,22,23]
	
	visionTable = r.camera.see()
	for marker in visionTable:
		if marker.id in home_table:
			moveByLocalVec(V(marker.cartesian.x - 1.5,marker.cartesian.z - 1.5))
			changeArmAngle(-89)
			return True
		else:
			turnByAngle(15,10)
			find_corner(arena)
			return False

def find_coord(markers):
    vision_table_full = get_vision_table()
    vision_table = [m for m in vision_table_full if m.is_wall_marker]
    if (len(vision_table)>=2):
        vec_x1 = vision_table[0].cartesian.x
        vec_y1 = vision_table[0].cartesian.z
        vec_x2 = vision_table[1].cartesian.x
        vec_y2 = vision_table[1].cartesian.z
        
        vec_to_1 = markers[vision_table[0].id]
        vec_to_2 = markers[vision_table[1].id]
        
        x1 = vec_to_1.position.xVal()
        y1 = vec_to_1.position.yVal()
        x2 = vec_to_2.position.xVal()
        y2 = vec_to_2.position.yVal()
        
                
        r1 = sqrt(vec_x1**2 + vec_y1**2)
        r2 = sqrt(vec_x2**2 + vec_y2**2)
        
        #d = sqrt((x1-x2)**2 + (y1-y2)**2)
        #l = (r1**2 - r2**2 + d**2)
        #h = sqrt(r2**2 - l**2)
        
        #x_coord = (l/d)*(x2-x1) + (h/d)(y2-y1) + x1
        #y_coord = (l/d)*(y2-y1) - (h/d)(x2-x1) + y1
        #vec_coord = V(x_coord, y_coord)
        #return vec_coord
        
        dx,dy = x2-x1,y2-y1
        d = sqrt(dx*dx+dy*dy)
        
        a = (r1*r1-r2*r2+d*d)/(2*d)
        h = sqrt(r1*r1-a*a)
        xm = x1 + a*dx/d
        ym = y1 + a*dy/d
        xs1 = xm + h*dy/d
        xs2 = xm - h*dy/d
        ys1 = ym - h*dx/d
        ys2 = ym + h*dx/d

        print(V(xs1,ys1))
        print(V(xs2,ys2))

    else:
        return False
            
            
                 
        
 # Running



def initMarkers():
	# Setup markers
	markerCodeTemp = 0
	
	for i in range(1, 7):  # Left
		markers.append(Marker(Vector(0, i), WALL, markerCodeTemp, norm=1.5 * pi))
		markerCodeTemp += 1

	for i in range(1, 8):  # Top markers
		markers.append(Marker(Vector(i, 8), WALL, markerCodeTemp))
		markerCodeTemp += 1

	for i in range(7, 0, -1):  # Right
		markers.append(Marker(Vector(8, i), WALL, markerCodeTemp, norm=0.5 * pi))
		markerCodeTemp += 1

	for i in range(7, 0, -1):  # Bottom
		markers.append(Marker(Vector(i, 0), WALL, markerCodeTemp, norm=1 * pi))
		markerCodeTemp += 1
		

	

	normArray = [1, 1.5, 0, 0.5]

	for position in [Vector(2, 4), Vector(4, 6), Vector(6, 4), Vector(4, 2)]:
		norm = 0
		for vectorToAdd in [Vector(-0.185,0), Vector(0, 0.185), Vector(0.185, 0), Vector(0, -0.185)]:
			markers.append(Marker(position + vectorToAdd, COLUMN, markerCodeTemp, norm=normArray[norm] * pi))
			markerCodeTemp += 1
			norm += 1

	# Token markers
	for i in range(0, 5):
		markers.append(Marker(None, TOKEN_ZONE_0, markerCodeTemp))
		markerCodeTemp += 1

	for i in range(0, 5):
		markers.append(Marker(None, TOKEN_ZONE_1, markerCodeTemp))
		markerCodeTemp += 1

	for i in range(0, 5):
		markers.append(Marker(None, TOKEN_ZONE_2, markerCodeTemp))
		markerCodeTemp += 1

	for i in range(0, 5):
		markers.append(Marker(None, TOKEN_ZONE_3, markerCodeTemp))
		markerCodeTemp += 1

#print markers

# 1-4 or false
overrideTeam = 1  #TL


# Should rename r to something else

markers = []
initMarkers()

r = Robot(wait_for_start_button=False)

robot = OurRobot()


if not overrideTeam:
	robotZone = r.zone
	pass
else:
	robotZone = overrideTeam-1



# in meters
startPositions = [V(0.5, 7.5), V(7.5, 7.5), V(7.5, 0.5), V(0.5, 0.5)]
startLookDirections = [135, 225, 315, 45]

scoringZones = [
	[V(1, 4), V(4, 4), V(4, 7), V(1, 7)],
	[V(4, 4), V(7, 4), V(7, 7), V(4, 7)],
	[V(4, 1), V(7, 1), V(7, 4), V(4, 4)],
	[V(1, 1), V(4, 1), V(4, 4), V(1, 4)]
]

tokenTypes = [
	TOKEN_ZONE_0,
	TOKEN_ZONE_1,
	TOKEN_ZONE_2,
	TOKEN_ZONE_3
]
tokenType = tokenTypes[robotZone]
oppositeTeam = robotZone + 2
if oppositeTeam > 3:
	oppositeTeam -= 4
oppositeTokenType = tokenTypes[oppositeTeam]




stopped = True

robotLock = threading.RLock()







# turnCameraServo(0)

scoringZone = scoringZones[robotZone]


robotPosition = startPositions[robotZone]
robotFacing = startLookDirections[robotZone]


# These currently are used really, but are ready for the future
robot.position = startPositions[robotZone]
robot.facing = startLookDirections[robotZone]



# Pin 2 is for the arm switches
r.servo_board.gpios[2].mode = PinMode.INPUT

# Alarm before raising arm, 3 second warning
r.power_board.buzz(0.4, frequency=200)
r.power_board.buzz(0.4, frequency=400)
r.power_board.buzz(0.4, frequency=200)
r.power_board.buzz(0.4, frequency=400)
r.power_board.buzz(0.4, frequency=200)
r.power_board.buzz(0.4, frequency=400)
r.power_board.buzz(0.4, frequency=200)
r.power_board.buzz(0.4, frequency=400)
time.sleep(3)

changeArmAngle(-89)
# changeArmAngle(-23)
# r.servo_board.servos[4].position = 0.6


r.power_board.wait_start()



	
def main():  # Just for testing really, in game just run playGame()
	#token = False
	#corner = False
	#moveByLocalVec(V(2,0))
	#while token == False:
		#token = find_token(2)
	#while corner == False:
		#corner = find_corner(2)
	while True:
		print(find_coord(markers))

main()
	
	


		
	
		


# TODO fix motors, test marker detection and location,...


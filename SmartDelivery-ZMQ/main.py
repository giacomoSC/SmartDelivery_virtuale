import math
from time import sleep
from zmqRemoteApi import RemoteAPIClient
import numpy as np
import cv2
import imutils

client = RemoteAPIClient()
sim = client.getObject('sim')

visorHandle = sim.getObjectHandle('/PioneerP3DX/cam0')
robot = sim.getObject('/PioneerP3DX')
leftMotor = sim.getObject('/PioneerP3DX/leftMotor')
rightMotor = sim.getObject('/PioneerP3DX/rightMotor')
intersectionI = sim.getObject('/intersectionI')
intersectionII = sim.getObject('/intersectionII')
st1 = sim.getObject('/wayst1')
st2 = sim.getObject('/wayst2')
nd1 = sim.getObject('/waynd1')
nd2 = sim.getObject('/waynd2')
rd1 = sim.getObject('/wayrd1')
rd2 = sim.getObject('/wayrd2')
th1 = sim.getObject('/wayth1')
th2 = sim.getObject('/wayth2')


class Controller:

    def __init__(self, shape):
        self.target = shape

    def check(self):
        if self.shape_recognizer() == self.target:
            return True
        return False

    def shape_recognizer(self):

        # reading image
        image, resolution = sim.getVisionSensorImg(visorHandle)
        uint8Numbers = sim.unpackUInt8Table(image)

        img = np.array(uint8Numbers, dtype=np.uint8)
        img.resize([resolution[0], resolution[1], 3])
        img = imutils.rotate_bound(img, 180)

        # converting image into grayscale image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # setting threshold of gray image
        _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # using a findContours() function
        contours, _ = cv2.findContours(
            threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        i = 0

        # list for storing names of shapes
        for contour in contours:

            # here we are ignoring first counter because
            # findcontour function detects whole image as shape
            if i == 0:
                i = 1
                continue

            # cv2.approxPloyDP() function to approximate the shape
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            # using drawContours() function
            cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

            # finding center point of shape
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])

            # putting shape name at center of each shape
            if len(approx) == 3:
                cv2.putText(img, 'Triangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                print('triangle')

                return 'triangle'

            elif len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h
                if 0.95 <= aspectRatio < 1.05:
                    cv2.putText(img, 'Square', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    print('square')

                    return 'square'
                else:
                    cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    print('rectangle')

                    return 'rectangle'
            elif len(approx) > 4:
                cv2.putText(img, 'circle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                print('circle')
                return 'circle'


def search_func(nw, shape):

    stopI = 0
    stopII = 0
    if nw == 1:
        stopI = st1
        stopII = st2
    elif nw == 2:
        stopI = nd1
        stopII = nd2
    elif nw == 3:
        stopI = rd1
        stopII = rd2
    elif nw == 4:
        stopI = th1
        stopII = th2

    result, dis, objectHandlePair = sim.checkDistance(robot, stopI, 0)
    eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
    yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])

    if nw % 2 == 1:
        while dis[6] > 0.04:
            result, dis, objectHandlePair = sim.checkDistance(robot, stopI, 0)
            sim.setJointTargetVelocity(rightMotor, 1)
            sim.setJointTargetVelocity(leftMotor, 1)

        while yawAngle > -1.5:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)

        if con.check():
            return 0

        while yawAngle < 1.5:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, 0.1)
            sim.setJointTargetVelocity(leftMotor, -0.1)

        if con.check():
            return 0

        while yawAngle > 0:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)

        result, dis, objectHandlePair = sim.checkDistance(robot, stopII, 0)
        while dis[6] > 0.04:
            result, dis, objectHandlePair = sim.checkDistance(robot, stopII, 0)
            sim.setJointTargetVelocity(rightMotor, 1)
            sim.setJointTargetVelocity(leftMotor, 1)

        while yawAngle > -1.5:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)

        if con.check():
            return 0

        while yawAngle < 1.5:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, 0.1)
            sim.setJointTargetVelocity(leftMotor, -0.1)

        if con.check():
            return 0

    else:

        while dis[6] > 0.04:
            result, dis, objectHandlePair = sim.checkDistance(robot, stopI, 0)
            sim.setJointTargetVelocity(rightMotor, 1)
            sim.setJointTargetVelocity(leftMotor, 1)

        while yawAngle > 1.5:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)

        if con.check():
            return 0

        while yawAngle > -1.5:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)

        if con.check():
            return 0

        while yawAngle < 0:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)

        result, dis, objectHandlePair = sim.checkDistance(robot, stopII, 0)
        while dis[6] > 0.04:
            result, dis, objectHandlePair = sim.checkDistance(robot, stopII, 0)
            sim.setJointTargetVelocity(rightMotor, 1)
            sim.setJointTargetVelocity(leftMotor, 1)

        while yawAngle > 1.5:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)

        if con.check():
            return 0

        while yawAngle > -1.5:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)

        if con.check():
            return 0

    return 1


sim.startSimulation()
sleep(1)

sim.setJointTargetVelocity(rightMotor, 0)
sim.setJointTargetVelocity(leftMotor, 0)

way = int(input("Way number:"))
destination = input("Destination shape:")

con = Controller(destination)
result, dis, objectHandlePair = sim.checkDistance(robot, intersectionI, 0)
eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])

while dis[6] > 0.04:
    result, dis, objectHandlePair = sim.checkDistance(robot, intersectionI, 0)
    sim.setJointTargetVelocity(rightMotor, 1)
    sim.setJointTargetVelocity(leftMotor, 1)

if way == 1:
    while yawAngle < 0:
        eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
        yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
        sim.setJointTargetVelocity(rightMotor, 0.1)
        sim.setJointTargetVelocity(leftMotor, -0.1)
    print(search_func(way, destination))
elif way == 2:
    while yawAngle < 3.00:
        eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
        yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
        sim.setJointTargetVelocity(rightMotor, -0.1)
        sim.setJointTargetVelocity(leftMotor, 0.1)
    print(search_func(way, destination))


if way == 3 or way == 4:
    result, dis, objectHandlePair = sim.checkDistance(robot, intersectionII, 0)

    while dis[6] > 0.04:
        result, dis, objectHandlePair = sim.checkDistance(robot, intersectionII, 0)
        sim.setJointTargetVelocity(rightMotor, 1)
        sim.setJointTargetVelocity(leftMotor, 1)

    if way == 3:
        while yawAngle < 0:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, 0.1)
            sim.setJointTargetVelocity(leftMotor, -0.1)
        print(search_func(way, destination))
    elif way == 4:
        while yawAngle < 3.00:
            eulerAngles = sim.getObjectOrientation(robot, sim.handle_world)
            yawAngle, pitchAngle, rollAngle = sim.alphaBetaGammaToYawPitchRoll(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            sim.setJointTargetVelocity(rightMotor, -0.1)
            sim.setJointTargetVelocity(leftMotor, 0.1)
        print(search_func(way, destination))

sim.stopSimulation()

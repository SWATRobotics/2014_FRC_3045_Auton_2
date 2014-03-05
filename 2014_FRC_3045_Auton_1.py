__author__ = 'Martin Haeberli'

# 2014 03 02
# sample code from git:
# https://github.com/FRC-Team-955/2014-Vision-Python/blob/master/src/vision.py

''' 2014 03 02 MPH Test OpenCV Python for FRC 2014 Team 3045 Autonomous mode
-todo:
-- get test file(s)
-- import and bind to NetworkTables
-- start running a test image analysis flow
-- tick / tock with NetworkTables?
-- Robot autonomous code to work with NetworkTables
-- track stuff from Chief Delphi - related to short delay after Autonomous starts
-- outline of Autonomous:
--- wait 1/2 second
--- move forward distance / time
--- identify vertical target(s)
--- identify hot target
--- go to firing position
--- (cock)
--- fire
'''

import cv2
import math
import numpy as np
from rectangle import Rectangle
from PIL import Image

import cPickle

def getFoundHotTarget():
    return foundHotTarget

def getFoundHorzTarget():
    return foundHorzTarget

def getDistance():
    return distance

def computeDistance(realHeight, targetHeight):
    return ((realHeight / targetHeight) * resHalfY) / math.tan(viewAngleVert * 3.14159 / 180.0)

def computeAngle(realHeight, targetHeight, distance):
    return math.atan(((realHeight / targetHeight) * resHalfY) / distance) * 180.0 / 3.14159

def drawRect(img, rect):
    #177
    cv2.rectangle(img, (int(rect.x), int(rect.y)), (int(rect.x + rect.width), int(rect.y + rect.height)), 255, 2)

def drawTargetX(img, rect):
    #redColor = np.array([200, 0, 0], np.uint8)
    cv2.circle(img, ( int((rect.x+(rect.width/2))), int(rect.y+(rect.height/2)) ), 2, (0,200,0), 2)

def round(value):
    return math.floor((value * 100) + 0.5) / 100

# from http://stackoverflow.com/questions/7722519/fast-rgb-thresholding-in-python-possibly-some-smart-opencv-code
def better_way(img_in):
    #img = img_in.convert('RGB')
    img = cv2.cvtColor(img_in, cv2.COLOR_BGR2RGB)
    #img = Image.open("rainbow.jpg").convert('RGB')
    arr = np.array(np.asarray(img))

    R = [(128,255),(200,255),(220,255)]
    red_range = np.logical_and(R[0][0] < arr[:,:,0], arr[:,:,0] < R[0][1])
    green_range = np.logical_and(R[1][0] < arr[:,:,0], arr[:,:,0] < R[1][1])
    blue_range = np.logical_and(R[2][0] < arr[:,:,0], arr[:,:,0] < R[2][1])
    valid_range = np.logical_and(red_range, green_range, blue_range)

    arr[valid_range] = 200
    arr[np.logical_not(valid_range)] = 0

    pil_image = Image.fromarray(arr)
    outim0 = np.array(pil_image)
    #outim = cv2.threshold(outim0, 127, 255, cv2.THRESH_BINARY)[1]
    return outim0
    #outim.save("rainbowout.jpg")


if __name__ == '__main__':

    #VideoCapture * cap = new  VideoCapture("http://root:pass@192.168.0.90/axis-cgi/mjpg/video.cgi?resolution=640x480.mjpg");
    #camera = cv2.VideoCapture(1)
    #camera = cv2.VideoCapture("http://10.0.1.169/mjpg/1/video.mjpg")
    #camera = cv2.VideoCapture("http://10.0.1.169/mjpg/1/video.cgi?resolution=640x480.mjpg")
    camera = cv2.VideoCapture("http://10.30.45.120/mjpg/1/video.mjpg")
    camera = cv2.VideoCapture("http://10.0.1.169/mjpg/1/video.mjpg")
    #camera = cv2.VideoCapture("http://10.0.1.169/axis-cgi/mjpg/video.cgi?resolution=640x480")
    foundHotTarget = False
    foundVertTarget = False
    viewAngle = 0.0 #needed but undefined; SWAG (real system uses gyro sensor)
    viewAngleVert = 19.832
    distance = 0.0
    resHalfY = 240
    horizTarget = Rectangle(0.0, 0.0, 23.5, 4.0)
    vertTarget = Rectangle(0.0, 0.0, 4.0, 32.0)
    debugMode = True

    if debugMode:
        cv2.namedWindow('color', cv2.WINDOW_NORMAL)
        #cv2.namedWindow('filtered', cv2.WINDOW_NORMAL)

    print "vision name", __name__

    while True :
        ret, img = camera.read() # img.shape 640x480 image
        #print "ret: " + str(ret) + "\n"
        if not ret : print "oops!\n"
        #cv2.imshow('input',img)

        # Convert to hsv img
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #
        rg = better_way(img)
        #cv2.imshow('huh', rg)
        #rg1 = cv2.cvtColor(rg, cv2.COLOR_BGR2HSV)
        hsv = cv2.cvtColor(rg, cv2.COLOR_BGR2HSV)

        #  Keep only green objects
        # higher s = less white, higher v = less black
        #invent sliders to adjust
        #lowerGreen = np.array([70, 70, 180])
        #upperGreen = np.array([110, 130,255])
        if False :
            lg_h, lg_s, lg_v = 0, 0, 90
            ug_h, ug_s, ug_v = 255, 255, 110
            #lg_h, lg_s, lg_v = 100, 6, 67
            #ug_h, ug_s, ug_v = 158, 98, 87
            lowerGreen = np.array([lg_h, lg_s, lg_v])
            upperGreen = np.array([ug_h, ug_s, ug_v])
            filteredGreen = cv2.inRange(hsv, lowerGreen, upperGreen)
            #cv2.imshow('fgreen',filteredGreen)

        filteredGreen = rg
        #cv2.cvtColor(rg, cv2.COLOR_RGB2GRAY, filteredGreen, 1)
        #cv2.cvtColor(filteredGreen, cv2.COLOR_GRAY2RGBA, filteredGreen, 1)
        #rg.convertTo(filteredGreen, CV_8UC1)
        #try as hard as heck to convert to happy format for findContours...
        min_t = np.array((199, 199, 199))
        max_t = np.array((201, 201, 201))
        filteredGreen = cv2.inRange(rg, min_t, max_t)
        #cv2.imshow('bigfilter',filteredGreen)

        # Filter out small objects
        filteredGreen = cv2.morphologyEx(filteredGreen, cv2.MORPH_OPEN, np.ones((3, 3)))
        #cv2.imshow('fgreens',filteredGreen)

        # Find all contours, counter is vector of points that are connected to make up a shape
        contours, hierarchy = cv2.findContours(filteredGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Resetting values
        foundHotTarget = False
        foundVertTarget = False

        # work around subtle bug - see StackOverflow
        # http://stackoverflow.com/questions/13337058/data-type-error-with-drawcontours-unless-i-pickle-unpickle-first
        tmp = cPickle.dumps(contours)
        contours = cPickle.loads(tmp)

        possibleHotTarget = Rectangle(0,0,0,0)
        possibleHotArea = possibleHotTarget.getArea()

        possibleVertTarget1 = Rectangle(0,0,0,0)
        possibleVertTarget2 = Rectangle(0,0,0,0)

        # Parse contours, calculate distance and check if it's the hot target
        for shape in contours:
            #target = Rectangle(*cv2.boundingRect(shape))
            x,y,w,h = cv2.boundingRect(shape)
            target = Rectangle(x,y,w,h)

            # Filter out small contours
            if target.getArea() > 300 and target.getArea() < 30000:

                # Compensate for horizontal view of the target
                #target.width = target.width / math.cos(viewAngle * math.PI / 180)
                #target.width = target.width / math.cos(viewAngle * 3.14159 / 180)

                # MPH improvements: - width / height (23.5/4) +-15% then possibly the hot target
                #  If the width of the target is greater than its height then it's probably the hot target
                hotTargetRatio = 23.5 / 4
                #hotTargetRatio = 5.3 # use this until calibrated
                hotTargetMargin = 0.25 # percent error margin
                if (target.width>target.height) and (target.height * (hotTargetRatio)*(1+hotTargetMargin)) >= target.width >= (target.height * ((hotTargetRatio)*(1-hotTargetMargin))):
                    foundHotTarget = True
                    if target.getArea() > possibleHotArea :
                        possibleHotTarget = target
                        possibleHotArea = target.getArea()
                    #distance = computeDistance(horizTarget.height, target.height)

                # MPH improvements: - height / width (32/4) +-15% then possibly the hot target
                # If the height of the target is greater than its width its probably a vert target
                vertTargetRatio = 32/4
                vertTargetMargin = 0.15
                if (target.height>target.width) and (target.width * (vertTargetRatio)*(1+vertTargetMargin)) >= target.height >= (target.width * ((vertTargetRatio)*(1-vertTargetMargin))):
                    foundVertTarget = True

                    #maintain up to two top-ranked vertical targets
                    if possibleVertTarget2.getArea() > possibleVertTarget1.getArea :
                        temp = possibleVertTarget1
                        possibleVertTarget1 = possibleVertTarget2
                        possibleVertTarget2 = temp
                    if target.getArea() > possibleVertTarget1.getArea() :
                        possibleVertTarget2 = possibleVertTarget1
                        possibleVertTarget1 = target
                    elif target.getArea() > possibleVertTarget1.getArea() :
                        possibleVertTarget2 = target

        if debugMode:
            if possibleHotArea > 0 :
                drawRect(img, possibleHotTarget)
                drawTargetX(img, possibleHotTarget)
                viewAngle = computeAngle(horizTarget.height, possibleHotTarget.height, 228)
                distanceHot = computeDistance(horizTarget.height, possibleHotTarget.height)
                #print "Distance: ", round(distanceHot), ", Hot Target", viewAngle, viewAngleVert, ", width: ", possibleHotTarget.width, ", height", possibleHotTarget.height
                print "Distance: ", round(distanceHot / 12), ", Hot Target", viewAngle, viewAngleVert, ", width: ", \
                    possibleHotTarget.width, ", height", possibleHotTarget.height

        if debugMode:
            if possibleVertTarget1.getArea() > 0 :
                drawRect(img, possibleVertTarget1)
                drawTargetX(img, possibleVertTarget1)
                viewAngle = computeAngle(vertTarget.height, possibleVertTarget1.height, 228)
                distanceVert1 = computeDistance(vertTarget.height, possibleVertTarget1.height)
                print "Distance: ", round(distanceVert1 / 12), ", Vert Target 1", viewAngle, viewAngleVert, ", width: ", \
                    possibleVertTarget1.width, ", height", possibleVertTarget1.height
            if possibleVertTarget2.getArea() > 0 :
                drawRect(img, possibleVertTarget2)
                drawTargetX(img, possibleVertTarget2)
                viewAngle = computeAngle(vertTarget.height, possibleVertTarget2.height, 228)
                distanceVert2 = computeDistance(vertTarget.height, possibleVertTarget2.height)
                print "Distance: ", round(distanceVert2 / 12), ", Vert Target 1", viewAngle, viewAngleVert, ", width: ", \
                    possibleVertTarget2.width, ", height", possibleVertTarget2.height

        if debugMode:
            cv2.imshow("color", img)
            cv2.waitKey(1)
            #cv2.imshow("filtered", filteredGreen)
            #cv2.waitKey(10)

        if cv2.waitKey(1) == 27:
            exit(0)




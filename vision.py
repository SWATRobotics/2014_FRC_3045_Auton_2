__author__ = 'Martin Haeberli'
# another test
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
#from pynetworktables import *
import arrow

import cPickle

def getFoundHotTarget():
    return foundHotTarget

#def getFoundHorzTarget():
#    return foundHorzTarget

def rHotTarget():
    return possibleHotTarget

def rVertTargets():
    return possibleVertTarget1, possibleVertTarget2

def getDistance():
    return distance

def computeDistance(realHeight, targetHeight):
    return ((realHeight / targetHeight) * resHalfY) / math.tan(viewAngleVert * 3.14159 / 180.0)

def computeAngle(realHeight, targetHeight, distance):
    return math.atan(((realHeight / targetHeight) * resHalfY) / distance) * 180.0 / 3.14159

def drawRect(img, rect):
    blueTargetColor = (255, 0, 0)
    cv2.rectangle(img, (int(rect.x), int(rect.y)), (int(rect.x + rect.width), int(rect.y + rect.height)), blueTargetColor, 2)

def drawXonTarget(img, rect):
    extraLength = 10
    redTargetColor = (0, 0, 200)
    targetX, targetY = rect.getCenter()
    cv2.circle(img, (targetX, targetY), 2, redTargetColor, 2)
    vlLeft = targetX
    vlBottom = targetY - (rect.height/2) - extraLength
    vlWidth = 0
    vlHeight = rect.height + 2 * extraLength
    cv2.rectangle(img, (vlLeft, vlBottom), (vlLeft+vlWidth, vlBottom+vlHeight), redTargetColor, 1)
    hlLeft = targetX - (rect.width/2) - extraLength
    hlBottom = targetY
    hlWidth = rect.width + 2 * extraLength
    hlHeight = 0
    cv2.rectangle(img, (hlLeft, hlBottom), (hlLeft+hlWidth, hlBottom+hlHeight), redTargetColor, 1)

def round(value):
    return math.floor((value * 100) + 0.5) / 100

# based on code samples at http://stackoverflow.com/questions/4890373/detecting-thresholds-in-hsv-color-space-from-rgb-using-python-pil
def rgb_to_hsl_hsv(a, isHSV=True):
    """
    Converts RGB image data to HSV or HSL.
    :param a: 3D array. Retval of np.asarray(Image.open(...), int)
    :param isHSV: True = HSV, False = HSL
    :return: H,S,L or H,S,V array
    """
    R, G, B = a.T

    m = np.min(a, 2).T
    M = np.max(a, 2).T

    C = M - m #chroma
    Cmsk = C != 0

    # Hue
    H = np.zeros(R.shape, int)
    mask = (M == R) & Cmsk
    H[mask] = np.mod(60 * (G[mask] - B[mask]) / C[mask], 360)
    mask = (M == G) & Cmsk
    H[mask] = (60 * (B[mask] - R[mask]) / C[mask] + 120)
    mask = (M == B) & Cmsk
    H[mask] = (60 * (R[mask] - G[mask]) / C[mask] + 240)
    H *= 255
    #cv2.imshow('hue',H.swapaxes(0, 1))
    H /= 360 # if you prefer, leave as 0-360, but don't convert to uint8


    # Saturation
    S = np.zeros(R.shape, int)

    if isHSV:
        # This code is for HSV:
        # Value
        V = M

        # Saturation
        S[Cmsk] = ((255 * C[Cmsk]) / V[Cmsk])
        #cv2.imshow('sat',S.swapaxes(0, 1))
        #cv2.imshow('val',V.swapaxes(0, 1))
        # H, S, and V are now defined as integers 0-255
        return H.swapaxes(0, 1), S.swapaxes(0, 1), V.swapaxes(0, 1)
    else:
        # This code is for HSL:
        # Value
        L = 0.5 * (M + m)

        # Saturation
        S[Cmsk] = ((C[Cmsk]) / (1 - np.absolute(2 * L[Cmsk]/255.0 - 1)))
        # H, S, and L are now defined as integers 0-255
        return H.swapaxes(0, 1), S.swapaxes(0, 1), L.swapaxes(0, 1)


# from http://stackoverflow.com/questions/7722519/fast-rgb-thresholding-in-python-possibly-some-smart-opencv-code
def better_way(img_in):
    #img = img_in.convert('RGB')
    img = cv2.cvtColor(img_in, cv2.COLOR_BGR2RGB)
    #img = Image.open("rainbow.jpg").convert('RGB')
    arr = np.array(np.asarray(img))
    #177 248 255
    R = [(170,190),(64,255),(250,256)]
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


#if __name__ == '__main__':

#VideoCapture * cap = new  VideoCapture("http://root:pass@192.168.0.90/axis-cgi/mjpg/video.cgi?resolution=640x480.mjpg");
#camera = cv2.VideoCapture(1)
#camera = cv2.VideoCapture("http://10.0.1.169/mjpg/1/video.mjpg")
#camera = cv2.VideoCapture("http://10.0.1.169/mjpg/1/video.cgi?resolution=640x480.mjpg")
#camera = cv2.VideoCapture("http://10.30.45.120/mjpg/1/video.mjpg")
#camera = cv2.VideoCapture("http://10.0.1.169/mjpg/1/video.mjpg")
#camera = cv2.VideoCapture("http://10.30.45.11/mjpg/1/video.mjpg")
#camera = cv2.VideoCapture("http://10.0.1.169/axis-cgi/mjpg/video.cgi?resolution=640x480")
foundHotTarget = False
foundVertTarget = False
viewAngle = 0.0 #needed but undefined; SWAG (real system uses gyro sensor)
viewAngleVert = 19.832
distance = 0.0
resHalfY = 240
horizTarget = Rectangle(0.0, 0.0, 23.5, 4.0)
vertTarget = Rectangle(0.0, 0.0, 4.0, 32.0)

#define here so a function can return the current rectangle
possibleHotTarget = Rectangle(0,0,0,0)
possibleVertTarget1 = Rectangle(0,0,0,0)
possibleVertTarget2 = Rectangle(0,0,0,0)

debugMode = True

if debugMode:
    cv2.namedWindow('color', cv2.WINDOW_NORMAL)
    #cv2.namedWindow('filtered', cv2.WINDOW_NORMAL)

#insert a comment
#insert another comment
print "vision name", __name__

def throttleValue(dist) :
    targetDistance = 10
    maxDistance = targetDistance + 2
    minDistance = targetDistance - 2
    result = 0
    if (dist > targetDistance + 1) :
        result = 0.5
    if (dist > maxDistance) :
        result = 1.0
    if (dist < targetDistance - 1) :
        result = 0.5
    if (dist < minDistance) :
        result = 1.0
    return result

def update(table, viewAngleHorz, deltaTime) :
    theta, dist = 0.0, 0.0
    #print "update()"

#    while True :
    #ret, img = camera.read() # img.shape 640x480 image
    #print "ret: " + str(ret) + "\n"
    ret = True
    img = cv2.imread("c:\Untitled.tiff")
    #if not ret : print "oops!\n"
    #elif
    #    cv2.imshow('input',img)

    # Convert to hsv img
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = rgb_to_hsl_hsv(img, True)
    #cv2.imshow("v", v)
    thresh = 255
    im_bw = cv2.threshold(v, 240, 255, cv2.THRESH_BINARY)[1]
    #cv2.imshow("im_bw",im_bw)

    #
    if False :
        rg = better_way(img)
        cv2.imshow('huh', rg)
    #rg1 = cv2.cvtColor(rg, cv2.COLOR_BGR2HSV)
    #hsv = cv2.cvtColor(rg, cv2.COLOR_BGR2HSV)

    #  Keep only green objects
    # higher s = less white, higher v = less black
    #invent sliders to adjust
    #lowerGreen = np.array([70, 70, 180])
    #upperGreen = np.array([110, 130,255])
    if False :
        lg_h, lg_s, lg_v = 0, 0, 254
        ug_h, ug_s, ug_v = 255, 255, 255
        #lg_h, lg_s, lg_v = 100, 6, 67
        #ug_h, ug_s, ug_v = 158, 98, 87
        lowerGreen = np.array([lg_h, lg_s, lg_v])
        upperGreen = np.array([ug_h, ug_s, ug_v])
        filteredGreen = cv2.inRange(hsv, lowerGreen, upperGreen)
        #cv2.imshow('fgreen',filteredGreen)

    if False :
        filteredGreen = rg
        #cv2.cvtColor(rg, cv2.COLOR_RGB2GRAY, filteredGreen, 1)
        #cv2.cvtColor(filteredGreen, cv2.COLOR_GRAY2RGBA, filteredGreen, 1)
        #rg.convertTo(filteredGreen, CV_8UC1)
        #try as hard as heck to convert to happy format for findContours...
        min_t = np.array((199, 199, 199))
        max_t = np.array((201, 201, 201))
        filteredGreen = cv2.inRange(rg, min_t, max_t)
    filteredGreen = im_bw
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
            hotTargetMargin = 0.25 # percent error margin
            if (target.width>target.height) and  (target.height * (hotTargetRatio)*(1+hotTargetMargin)) >= target.width >= (target.height * ((hotTargetRatio)*(1-hotTargetMargin))):
                foundHotTarget = True
                if target.getArea() > possibleHotArea :
                    possibleHotTarget = target
                    possibleHotArea = target.getArea()
                #distance = computeDistance(horizTarget.height, target.height)

            # MPH improvements: - height / width (32/4) +-15% then possibly the hot target
            # If the height of the target is greater than its width its probably a vert target
            vertTargetRatio = 32 / 4
            vertTargetMargin = 0.30
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

    #vt1loc = (50,440)
    #vtrloc = (580,440)
    #cv2.putText(img,"Hello World!!!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
    #img, text, textOrg, fontFace, fontScale, thickness, baseline?
    #cv2.putText
    #cv2.putText(img, "Hi again", vtlloc, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), thickness=2)
    #lineType= cv2.CV_AA

    if debugMode:
        if possibleHotArea > 0 :
            drawRect(img, possibleHotTarget)
            drawXonTarget(img, possibleHotTarget)
            viewAngle = computeAngle(horizTarget.height, possibleHotTarget.height, 228)
            distanceHot = computeDistance(horizTarget.height, possibleHotTarget.height)

            #print "Distance: ", round(distanceHot), ", Hot Target", viewAngle, viewAngleVert, ", width: ", possibleHotTarget.width, ", height", possibleHotTarget.height
            #print "Distance: ", round(distanceHot / 12), ", Hot Target", viewAngle, viewAngleVert, ", width: ", \
            #    possibleHotTarget.width, ", height", possibleHotTarget.height
    haveHotTarget = False
    if possibleHotArea > 0 :
        haveHotTarget = True
    table.PutBoolean('hotTarget', haveHotTarget)

    if debugMode:
        lString = "L:"
        rString = "R:"
        v1DistStr = ""
        v2DistStr = ""
        if possibleVertTarget1.getArea() > 0 :
            drawRect(img, possibleVertTarget1)
            drawXonTarget(img, possibleVertTarget1)
            viewAngle = computeAngle(vertTarget.height, possibleVertTarget1.height, 228)
            distanceVert1 = computeDistance(vertTarget.height, possibleVertTarget1.height)
            distanceVert1AsFeet = distanceVert1 / 12
            #f(x)= 1.29x -0.09 - per Nathan
            #distanceVert1AsFeet = (1.29 * distanceVert1AsFeet) - 0.09
            v1DistStr = str(round(distanceVert1AsFeet))
            #print "Distance: ", round(distanceVert1 / 12), ", Vert Target 1", viewAngle, viewAngleVert, ", width: ", \
            #    possibleVertTarget1.width, ", height", possibleVertTarget1.height
        if possibleVertTarget2.getArea() > 0 :
            drawRect(img, possibleVertTarget2)
            drawXonTarget(img, possibleVertTarget2)
            viewAngle = computeAngle(vertTarget.height, possibleVertTarget2.height, 228)
            distanceVert2 = computeDistance(vertTarget.height, possibleVertTarget2.height)
            distanceVert2AsFeet = distanceVert2 / 12
            #f(x)= 1.29x -0.09 - per Nathan
            #distanceVert2AsFeet = (1.29 * distanceVert2AsFeet) - 0.09
            v2DistStr = str(round(distanceVert2/12))
            #print "Distance: ", round(distanceVert2 / 12), ", Vert Target 1", viewAngle, viewAngleVert, ", width: ", \
            #    possibleVertTarget2.width, ", height", possibleVertTarget2.height

        if (possibleVertTarget1.getArea()>0) and (possibleVertTarget1.getArea()>0) :
            if (possibleVertTarget1.x+(possibleVertTarget1.width)/2) > (possibleVertTarget2.x+(possibleVertTarget2.width)/2) :
                tmpStr = v2DistStr
                v2DistStr = v1DistStr
                v2DistStr = tmpStr

        if (possibleVertTarget1.getArea()>0) :
            dist = distanceVert1
        lString = lString + v1DistStr
        rString = rString + v2DistStr
        fpsString = "FPS: "
        fpsString = fpsString + str(round(1/deltaTime))
        cv2.putText(img, lString, (10, 440), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), thickness=2)
        cv2.putText(img, rString, (510, 440), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), thickness=2)
        cv2.putText(img, fpsString, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), thickness=2)

    if debugMode:
        arrow.draw_arrow(img, (320, 240), (100, 150), (100, 0, 255), 20);
        throttleArrow = throttleValue(dist)
        arrow.draw_arrow(img, (320, 430), (320, int(430 - (180*throttleArrow))), (200, 0, 255), 20);

        cv2.imshow("color", img)
        cv2.waitKey(1)
        #cv2.imshow("filtered", filteredGreen)
        #cv2.waitKey(10)

    if cv2.waitKey(1) == 27:
        exit(0)

    return theta, dist


'''
' Here will be the code for obstacle detecting
'
'''

''' code below is from https://www.learnopencv.com/blob-detection-using-opencv-python-c/,
' http://introlab.github.io/find-object/, https://www.intorobotics.com/how-to-detect-and-track-object-with-opencv/
' Make sure to install imutils (pip install imutils) for object detector
'''
import numpy as np
import math
from imutils import contours
from imutils import perspective
from imutils import *
from cv2 import *
from argparse import *
from RobotWorld import *
import sys
import random
from sensor_msgs.msg import CameraInfo
sys.path.append("/home/kinova/MillenCapstone/MadalynMillenCapstone/vision_opencv-melodic/image_geometry/src/image_geometry")
import cameramodels

#https://www.pyimagesearch.com/2016/03/28/measuring-size-of-objects-in-an-image-with-opencv/
def midpoint(point1, point2):
    return ((point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5)

def takePhoto():
    argpar = ArgumentParser()
    argpar.add_argument("-i", "--image", required=False,
        help="path to the input image")
    '''argpar.add_argument("-w", "--start", type=int, required=False,
        help="starting point of the path)")
    argpar.add_argument("-w", "--end", type=int, required=False,
        help="ending point of the path)")'''
    args = vars(argpar.parse_args())
    #https://stackoverflow.com/questions/11094481/capturing-a-single-image-from-my-webcam-in-java-or-python
    camera = VideoCapture(0)
    '''cameraModel = cameramodels.PinholeCameraModel()
    cameraInfo = cameraModel.fromCameraInfo(CameraInfo())'''
    saved, image = camera.read()
    if saved:
        namedWindow("Camera")
        imshow("Camera", image)
        destroyWindow("Camera")
        if not imwrite("workSpace.jpg", image):
            raise Exception("Could not write image")
        camera.release()
    if len(args) > 0:
        if not (args["image"] is None):
            image = imread(args["image"])
    '''newImage = imread(args["image"])
    cameraModel.rectifyImage(image, newImage)'''
    return args, image #newImage

def detectAndDraw(args, image):
    color = cvtColor(image, COLOR_BGR2GRAY)
    color = GaussianBlur(color, (7,7), 0)

    edgeDetection =  Canny(color, 50, 100)
    edgeDetection = dilate(edgeDetection, None, iterations=1)
    edgeDetection = erode(edgeDetection, None, iterations=1)

    contors = findContours(edgeDetection.copy(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
    if contors is None or len(contors) == 0:
        raise Exception("No obstacles detected")
    contors = grab_contours(contors)

    (contors, _) = contours.sort_contours(contors)
    newImage = None
    listOfPoints = []
    obstacleInfo = []
    boxPointsList = []
    counter = 1
    for cnts in contors:
        contourBoxes(image, listOfPoints, boxPointsList, obstacleInfo, counter, cnts)
        counter += 1
    return listOfPoints, obstacleInfo, boxPointsList

def contourBoxes(image, listOfPoints, boxPointsList, obstacleInfo, obstacleNum, cnts):
    if contourArea(cnts) < 100:
        return

    original = image.copy()
    box = minAreaRect(cnts)
    box = boxPoints(box)
    box = np.array(box, dtype="int")

    box = perspective.order_points(box)
    boxPointsList.append(box)
    (topLeft, topRight, botRight, botLeft) = box
    newPoints = []
    x1, y1 = topLeft / 20
    x2, y2 = topRight / 20
    x3, y3 = botRight / 20
    x4, y4 = botLeft / 20
    angle = math.atan((y2-y1)/(x2-x1))*180/math.pi #https://pdnotebook.com/measuring-angles-in-opencv-2f0551b8dd5a
    print (angle)
    width = sqrt( (x2-x1)**2 + (y2-y1)**2 )
    print(width[0])
    height = sqrt( (x3-x2)**2 + (y3-y2)**2 )
    print ((topLeft / 20))
    print (height[0])
    r, g, b = [random.random() for i in range(3)]
    color = r, g, b, 1

    obstacle = Obstacle("Obstacle " + str(obstacleNum), (x4, y4), width[0], height[0], color, angle)
    obstacleInfo.append(obstacle)

    listOfPoints.append(getLine(x1, y1, x2, y2))
    listOfPoints.append(getLine(x2, y2, x3, y3))
    listOfPoints.append(getLine(x3, y3, x4, y4))
    listOfPoints.append(getLine(x1, y1, x4, y4))
    drawContours(original, [box.astype("int")], -1, (0,0,0), 2)
    imshow("image", original)
    waitKey(300)

#Code taken from https://stackoverflow.com/questions/25837544/get-all-points-of-a-straight-line-in-python
def getLine(x1, y1, x2, y2):
    allPoints = []
    isStep = abs(y2 - y1) > abs(x2 - x1)
    if isStep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    isReversed = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        isReversed = True
    deltaX = x2 - x1
    deltaY = abs(y2 - y1)
    error = int(deltaX / 2)
    y = y1
    if y1 < y2:
        yStep = 1
    else:
        yStep = -1
    for x in range(int(x1), int(x2 + 1)):
        if isStep:
            allPoints.append((y, x))
        else:
            allPoints.append((x, y))
        error -= deltaY
        if error < 0:
            y += yStep
            error += deltaX
    if isReversed:
        allPoints.reverse()
    return allPoints

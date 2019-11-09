'''
' Here will be the code for obstacle detecting
'
'''

''' code below is from https://www.learnopencv.com/blob-detection-using-opencv-python-c/,
' http://introlab.github.io/find-object/, https://www.intorobotics.com/how-to-detect-and-track-object-with-opencv/
' Make sure to install imutils (pip install imutils) for object detector
'''
import numpy as np
from scipy.spatial import distance as dist
from imutils import contours
from imutils import perspective
from imutils import *
from cv2 import *
from argparse import *
from collections import deque

#https://www.pyimagesearch.com/2016/03/28/measuring-size-of-objects-in-an-image-with-opencv/
def midpoint(point1, point2):
    return ((point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5)

def takePhoto():
    argpar = ArgumentParser()
    argpar.add_argument("-i", "--image", required=True,
        help="path to the input image")
    argpar.add_argument("-w", "--width", type=float, required=True,
        help="width of the left-most object in the image (in inches)")
    args = vars(argpar.parse_args())
    #https://stackoverflow.com/questions/11094481/capturing-a-single-image-from-my-webcam-in-java-or-python
    camera = VideoCapture(0)
    saved, image = camera.read()
    if saved:
        namedWindow("Camera")
        imshow("Camera", image)
        destroyWindow("Camera")
        if not imwrite("workSpace.jpg", image):
            raise Exception("Could not write image")
        camera.release()
    image = imread(args["image"])
    return args, image

def detectAndDraw(args, image):
    color = cvtColor(image, COLOR_BGR2GRAY)
    color = GaussianBlur(color, (7,7), 0)

    edgeDetection =  Canny(color, 50, 100)
    edgeDetection = dilate(edgeDetection, None, iterations=1)
    edgeDetection = erode(edgeDetection, None, iterations=1)

    contors = findContours(edgeDetection.copy(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
    contors = grab_contours(contors)

    (contors, _) = contours.sort_contours(contors)

    listOfPoints = []
    for cnts in contors:
        contourBoxes(args, image, listOfPoints, cnts)

    return listOfPoints

def contourBoxes(args, image, listOfPoints, cnts):
    if contourArea(cnts) < 100:
        return

    original = image.copy()
    box = minAreaRect(cnts)
    box = cv.BoxPoints(box) if is_cv2() else boxPoints(box)
    box = np.array(box, dtype="int")

    box = perspective.order_points(box)
    (topLeft, topRight, botRight, botLeft) = box
    x1, y1 = topLeft
    x2, y2 = topRight
    x3, y3 = botRight
    x4, y4 = botLeft
    listOfPoints.append(getLine(x1, y1, x2, y2))
    listOfPoints.append(getLine(x2, y2, x3, y3))
    listOfPoints.append(getLine(x3, y3, x4, y4))
    listOfPoints.append(getLine(x1, y1, x4, y4))
    drawContours(original, [box.astype("int")], -1, (0,0,0), 2)

    imshow("image", original)
    waitKey(300)

#https://stackoverflow.com/questions/25837544/get-all-points-of-a-straight-line-in-python
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
    return allP

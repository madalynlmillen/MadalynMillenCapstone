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
import imutils
from cv2 import *
from argparse import *


#https://www.pyimagesearch.com/2016/03/28/measuring-size-of-objects-in-an-image-with-opencv/
def midpoint(point1, point2):
    return ((point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5)

argpar = ArgumentParser()
argpar =
#https://stackoverflow.com/questions/11094481/capturing-a-single-image-from-my-webcam-in-java-or-python
camera = VideoCapture(0)
saved, image = camera.read()
if saved:
    namedWindow("Camera")
    imshow("Camera", image)
    #destroyWindow("Camera")
    if not imwrite("workSpace.jpg", image):
        raise Exception("Could not write image")
    camera.release()

color = cvtColor(image, COLOR_BGR2GRAY)
color = GaussianBlur(color, (7,7), 0)

edgeDetection =  Canny(color, 50, 100)
edgeDetection = dilate(edgeDetection, None, iterations=1)
edgeDetection = erode(edgeDetection, None, iterations=1)

contors = findContours(edgeDetection.copy(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
contors = imutils.grab_contours(contors)
'''
#(coun)
# Read image
image = imread("stakeTest4.jpg", IMREAD_GRAYSCALE) #don't forget to alter code to read new images taken
# Setup SimpleBlobDetector parameters.
ret, thresh = threshold(image, 127, 255, 0)
contours = findContours(thresh, 1, 2)

cont = contours[0]
m = moments(cont)
print (m)
params = SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area.
params.filterByArea = True
params.minArea = 1500

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.87

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.01

# Create a detector with the parameters
ver = imutils.check_opencv_version("2.")
if ver:
    detector = SimpleBlobDetector(params)
else:
    detector = SimpleBlobDetector_create(params)

# Detect blobs
keypoints = detector.detect(image)
# Draw detected blobs as red circles. cv2 ensures the circle corresponds to the size of the blob
imageKP = drawKeypoints(image, keypoints, np.array([]), (0,0,255), DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# Show keypoints
imshow("demo", imageKP)
waitKey(0)
#End of Borrowed Code
class Triangle:
    def __init__(self, point1, point2, point3):
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
    def getPerimeter(self):
        return 0

class Square:
    def __init__(self, point1, point2, point3, point4):
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
        self.point4 = point4
    def getPerimeter(self):
        return 0

class Circle:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
    def getCircumfrence(self):
        return 2 * math.pi * self.radius


'''
#Code inspired by https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
'''
class Detector:
    def __init__(self):
        pass
    def detect(self, contour):
        shape = None
        perimeter = arcLength(contour, True)
        approxOfShape = approxPolyDP(contour, 0.04 * perimeter, True)

        if approxOfShape == 3:
            shape = Triangle((0,0), (0,0), (0,0))
        if approxOfShape == 4:
            shape = Square((0,0), (0,0), (0,0), (0,0))
        else:
            shape = Circle((0,0), 0)
        return shape'''

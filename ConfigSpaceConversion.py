import numpy as np
from imutils import *
from cv2 import *
import BlobDetection
import math

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

def convertObjects():
    listOfObjects = BlobDetection.detectAndDraw()
    points = []
    return points

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
        return shape
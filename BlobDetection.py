'''
' Here will be the code for obstacle detecting
'
'''

''' code below is from https://www.learnopencv.com/blob-detection-using-opencv-python-c/,
' http://introlab.github.io/find-object/, https://www.intorobotics.com/how-to-detect-and-track-object-with-opencv/
' Make sure to install imutils (pip install imutils) for object detector
'''
import cv2
import numpy as np

# Read image
image = cv2.imread("3DBlob.jpg", cv2.IMREAD_GRAYSCALE) #don't forget to alter code to read new images taken
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

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
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3:
    detector = cv2.SimpleBlobDetector(params)
else:
    detector = cv2.SimpleBlobDetector_create(params)
# Detect blobs
keypoints = detector.detect(image)
# Draw detected blobs as red circles. cv2 ensures the circle corresponds to the size of the blob
imageKP = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# Show keypoints
cv2.imshow("Keypoints", imageKP)
cv2.waitKey(0)
''' End of Borrowed Code'''
class Triangle:
    def __init__(self, point1, point2, point3):
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3

class Square:
    def __init__(self, point1, point2, point3, point4):
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
        self.point4 = point4

class Circle:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius


'''
' Code inspired by https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
'''
class Detector:
    def __init__(self):
        pass
    def detect(self, contour):
        shape = None
        perimeter = cv2.arcLength(contour, True)
        approxOfShape = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

        if approxOfShape == 3:
            shape = Triangle((0,0), (0,0), (0,0))
        if approxOfShape == 4:
            shape = Triangle((0,0), (0,0), (0,0), (0,0))
        else:
            shape = Circle((0,0), 0)
        return shape
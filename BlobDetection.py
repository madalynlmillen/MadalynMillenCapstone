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
    #destroyWindow("Camera")
    if not imwrite("workSpace.jpg", image):
        raise Exception("Could not write image")
    camera.release()

image = imread(args["image"])
color = cvtColor(image, COLOR_BGR2GRAY)
color = GaussianBlur(color, (7,7), 0)

edgeDetection =  Canny(color, 50, 100)
edgeDetection = dilate(edgeDetection, None, iterations=1)
edgeDetection = erode(edgeDetection, None, iterations=1)

contors = findContours(edgeDetection.copy(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
contors = grab_contours(contors)

(contors, _) = contours.sort_contours(contors)
pixelsPerMetric = None

for cnts in contors:
    if contourArea(cnts) < 100:
        continue

    original = image.copy()
    box = minAreaRect(cnts)
    box = cv.BoxPoints(box) if is_cv2() else boxPoints(box)
    box = np.array(box, dtype="int")

    box = perspective.order_points(box)
    drawContours(original, [box.astype("int")], -1, (0,255,0), 2)

    for (x,y) in box:
        circle(original, (int(x), int(y)), 5, (0,0,255), -1)

    (topLeft, topRight, bottomRight, bottomLeft) = box
    (topX, topY) = midpoint(topLeft, topRight)
    (bottomX, bottomY) = midpoint(bottomLeft, bottomRight)

    (topBotLX, topBotLY) = midpoint(topLeft, bottomLeft)
    (botTopRX, botTopRY) = midpoint(topRight, bottomRight)

    circle(original, (int(topX), int(topY)), 5, (255,0,0), -1)
    circle(original, (int(bottomX), int(bottomY)), 5, (255,0,0), -1)
    circle(original, (int(topBotLX), int(topBotLY)), 5, (255,0,0), -1)
    circle(original, (int(botTopRX), int(botTopRY)), 5, (255,0,0), -1)

    line(original, (int(topX), int(topY)), (int(bottomX), int(bottomY)), (255,0,255), 2)
    line(original, (int(topBotLX), int(topBotLY)), (int(botTopRX), int(botTopRY)), (255,0,255), 2)

    dimension1 = dist.euclidean((topX, topY), (bottomX, bottomY))
    dimension2 = dist.euclidean((topBotLX, topBotLY), (botTopRX, botTopRY))

    if pixelsPerMetric is None:
        pixelsPerMetric = dimension2 / args["width"]

    dimension1 = dimension1 / pixelsPerMetric
    dimension2 = dimension2 / pixelsPerMetric

    putText(original, "{:.1f}in".format(dimension1),(int(topX - 15), int(topY - 10)), FONT_HERSHEY_SIMPLEX,0.65, (255, 255, 255), 2)
    putText(original, "{:.1f}in".format(dimension2),(int(botTopRX + 10), int(botTopRY)), FONT_HERSHEY_SIMPLEX,0.65, (255, 255, 255), 2)

    imshow("image", original)
    waitKey(0)
'''Code from Robotics Git'''
'''mask = inRange(color, )
mask = erode(mask, None, iterations=2)
mask = dilate(mask, None, iterations=2)

contors = contours.findContours(mask, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)[-2]
center = None


if len(contors) > 0:
    cnts = max(contors, key=contourArea)
    ((x,y), radius) = minEnclosingCircle(cnts)
    mom = moments(cnts)
    center = (int(mom["m10"] / mom["m00"]), int(mom["m01"] / mom["m00"]))

points = deque
points.appendleft(center)

for i in range(1, len(points)):
    if points[i - 1] is None or points[i] is None:
        continue
    thickness = int(np.sqrt())
''''''End of Robotics Git Code''''''
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
params.minArea = 2500

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
ver = check_opencv_version("2.")
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

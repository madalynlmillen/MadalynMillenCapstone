import sys
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from PIL import Image
from cv2 import *
import BlobDetection

'''
#Code inspired by https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
'''

'''
This function takes the list of obstacle points collected by BlobDetection and
creates a list of boolean values. This list will assign a spot as false if there is an obstacle
there, and be true otherwise. This list will then be passed to the path planning function.
'''
def convertObjects():
    args, image = BlobDetection.takePhoto()
    listOfObjects, obstacleInfo, boxPointsList = BlobDetection.detectAndDraw(args, image)
    imageList = np.array(image, dtype='float')

    width = len(imageList)
    height = len(imageList[0])

    imageBoolList = np.ones((width, height), dtype=bool)
    newSpaceImage = np.zeros((width, height, 3), dtype=np.uint8)
    #This loop assigns a false value to spots with obstacles in them and helps create an image that tells where the obstacles are
    for j in listOfObjects:
        for pair in j:
            a,b = pair
            imageBoolList[int(b)][int(a)] = False
            newSpaceImage[int(b)][int(a)] = [255, 255, 255]

    #This image is created from the boolean list to show where the obstacles are
    newSpaceImage = Image.fromarray(newSpaceImage, 'RGB')
    newSpaceImage.save('newSpaceImage.jpg')
    newSpaceImage = imread('newSpaceImage.jpg')
    imshow('try', newSpaceImage)
    waitKey(10000)
    return imageBoolList, obstacleInfo

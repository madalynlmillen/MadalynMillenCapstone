import sys
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from PIL import Image
from cv2 import *
import BlobDetection
import math

'''
#Code inspired by https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
'''

def convertObjects():
    args, image = BlobDetection.takePhoto()
    listOfObjects = BlobDetection.detectAndDraw(args, image)
    print (len(listOfObjects) * len(listOfObjects[0]))
    imageList = np.array(image, dtype='float')
    width = len(imageList)
    height = len(imageList[0])
    imageBoolList = np.ones((width, height), dtype=bool)
    newSpaceImage = np.zeros((width, height, 3), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            for j in listOfObjects:
                for a,b in j:
                    if x == a and y == b:
                        imageBoolList[y][x] = False
                        newSpaceImage[y][x] = [255,255,255]
    newSpaceImage = Image.fromarray(newSpaceImage, 'RGB')
    newSpaceImage.save('newSpaceImage.jpg')
    newSpaceImage = imread('newSpaceImage.jpg')
    imshow('try', newSpaceImage)
    waitKey(10000)
    return imageBoolList

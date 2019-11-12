import sys
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from PIL import Image
from cv2 import *
import BlobDetection

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
    count = 0
    for j in listOfObjects:
        for pair in j:
            a,b = pair
            print pair
            count += 1
            imageBoolList[int(b)][int(a)] = False
            newSpaceImage[int(b)][int(a)] = [255, 255, 255]
    print count
    newSpaceImage = Image.fromarray(newSpaceImage, 'RGB')
    newSpaceImage.save('newSpaceImage.jpg')
    newSpaceImage = imread('newSpaceImage.jpg')
    imshow('try', newSpaceImage)
    waitKey(10000)
    return imageBoolList

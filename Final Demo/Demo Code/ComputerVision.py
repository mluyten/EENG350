# Group 6 | EENG350 | Demo 1 Computer Vision Code
# Purpose: Detects an Aruco marker and determines the angle between the camera and the Aruco marker

import time
import cv2
import math
import threading
import io
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from PIL import Image
import matplotlib.pyplot as plt

# This was defined as a class in order to make it easier to access the functions from the other Python Files
class ComputerVision():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.runCapture = False;
        self.image = None;
        
    def startCapture(self):
        self.runCapture = True
        thread = threading.Thread(target=self.capture)
        thread.start()
            
    def capture(self):
        stream = io.BytesIO()
        while self.runCapture:
            ret, frame = self.cap.read()
            self.image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
    def stopCapture(self):
        self.runCapture = False;

    def arucoDetect(self, image, showImage=False):
        
        # verify that the supplied ArUCo tag exists and is supported by
        # OpenCV
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
                                                           parameters=arucoParams)
        if ids.all() != None:
            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            h, w = image.shape 
            focalLength = 912.916 #Value found expiermentally 
            arucoHeight = 100  # in mm
            pixelHeight = ((topRight[1] - bottomRight[1]) + (topLeft[1] - bottomLeft[1])) / 2 #Take average of the top two corners just so it gets the average height
            realDistance = focalLength * arucoHeight / pixelHeight #finds the distance based on the math that Cam showed us
            if cX < w/2:
                width = realDistance * ((w / 2) - cX) / focalLength
                width = 0.0393701 * width
                realDistance = 0.0393701 * realDistance
                return [realDistance, width, -1]
            else:
                width = realDistance * (cX - (w / 2)) / focalLength
                width = 0.0393701 * width
                realDistance = 0.0393701 * realDistance
                return [realDistance, width, 1]
        else:
            return -1
    # There is a lot of comments that live within this function already, but I would like to expand on that.
    # Essentially we start out by determining if there is an AruCo marker in the image. It does edge detection and
    # checks to see if the Aruco in the image is within any of the dictionaries supplied. I currently only have one
    # dictionary supplied in order help cut down on the time the program needs to check to see if it is supplied, can be
    # expanded at any point though. A lot of this code comes from https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
    # However there was some modifying done in order to help this work faster. I removed the function that shows the image for this time, but it can be done whenever needed
    # by calling the function and setting it to show image true. I also made it so if it doesn't find an aruco marker, it doesn't fail, like the code given at the website.
    # I also make it so it works with images that are spit out from the camera, rather than saved images, this will help keep us from filling up the memory on the pi.
    # I also then adjusted it using the precalculated center of the AruCo marker and compared that to the total size of the image and used that for the determinations of what
    # quadrant the image is in.

    def getAngleAndDistance(self):
        return self.arucoDetect(self.image)
        #return self.arucoDetect(self.resize(self.convertToGray(self.takeImage())))
        # This function is the one that is called within the file that communicates with the Arduino. It is kinda messy so I have set it up so the call is easy
        # within the greater function of the project.

    def setWhiteBalance(self):
        g1 = self.camera.awb_gains
        time.sleep(0.5)
        g2 = self.camera.awb_gains
        time.sleep(0.5)
        g3 = self.camera.awb_gains
        self.camera.awb_mode = 'off'
        gainAvg = (g1[0] / 3 + g2[0] / 3 + g3[0] / 3, g1[1] / 3 + g2[1] / 3 + g3[1] / 3)
        self.camera.awb_gains = gainAvg
        # This function is run whenever we initialize the project. It takes 3 images pretty close together and then averages the gains
        # from them in order to set the white balance so we don't have any issues when it comes to stray colors and odditites in our
        # pictures.
    
    def arucoExist(self):
        #image = self.convertToGray(image)
        #image = self.resize(self.convertToGray(self.takeImage()))
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        # verify that the supplied ArUCo tag exists and is supported by
        # OpenCV
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.image, arucoDict,
                                                          parameters=arucoParams)
        
        try:
            for id in ids:
                if id >= 0 and id < 8:
                    return True
            return False
        except:
            return False

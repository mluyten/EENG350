import time
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
class computer_vision():
    def __init__(self):
        self.camera = PiCamera()
        self.setWhiteBalance()

    def takeImage(self):
        # initialize the camera and grab a reference to the raw camera capture
        rawCapture = PiRGBArray(self.camera)
        # allow the camera to warmup
        time.sleep(0.1)
        # grab an image from the camera
        #print("Capturing Image...")
        try:
            self.camera.capture(rawCapture, format="bgr")
            image = rawCapture.array
            return image
        except:
            print("Failed to capture")
            # display the image on screen and wait for a keypress

    def convertToGray(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return gray

    def resize(self, gray):
        resizeImg = cv2.resize(gray, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
        return resizeImg

    def arucoDetect(self, resize, showImage=False):
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        image = resize
        # verify that the supplied ArUCo tag exists and is supported by
        # OpenCV
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
                                                       parameters=arucoParams)
        try:
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
    # draw the bounding box of the ArUCo detection
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
    # compute and draw the center (x, y)-coordinates of the ArUco
    # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
    # draw the ArUco marker ID on the image
            cv2.putText(image, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
            #print("[INFO] ArUco marker ID: {}".format(markerID))

            h, w = image.shape
            if (cX < int(w / 2)) and cY < int(h / 2):
                arucoLocation = "NW"
            elif (cX > int(w / 2) and cY < int(h / 2)):
                arucoLocation = "NE"
            elif (cX < int(w / 2) and cY > int(h / 2)):
                arucoLocation = "SW"
            else:
                arucoLocation = "SE"
            if showImage:
                cv2.imshow(image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                #print(arucoLocation)
            return arucoLocation 
        except:
            #print("No Aruco Detected")
            return "NA"
    #This is what needs to be called
    def getArucoQuadrant(self):
        return self.arucoDetect(self.resize(self.convertToGray(self.takeImage())))

    def setWhiteBalance(self):
        g1 = self.camera.awb_gains
        time.sleep(0.5)
        g2 = self.camera.awb_gains
        time.sleep(0.5)
        g3 = self.camera.awb_gains
        self.camera.awb_mode = 'off'
        gainAvg = (g1[0]/3 + g2[0]/3 + g3[0]/3, g1[1]/3 + g2[1]/3 + g3[1]/3)
        self.camera.awb_gains = gainAvg

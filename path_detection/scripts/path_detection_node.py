#!/usr/bin/env python
import math
import rospy
import numpy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist



class PathDetector:
    def __init__(self, pub):
        self.pub = pub
        self.initialized = 0
        self.center = []
        self.wp = 0

    def getPathColor(self, img): 
        #filtrate image
        imgFil = cv2.medianBlur(img, 3)

        #extract part of the image in the close area of camera
        [sizeY, sizeX, sizeColor] =  imgFil.shape
        centerImg = imgFil[ (sizeY - int(sizeY/9)) : sizeY , int(sizeX/2 - sizeX/4.5) : int(sizeX/2 + sizeX/4.5), :]

        # convert BGR colorspace of proccesed image to HSV
        hsv = cv2.cvtColor(imgFil, cv2.COLOR_BGR2HSV)

        #convert img array from uint8t [y,x,3] to float32t [y*x,3]
        height, width, _ = centerImg.shape
        centerIm = numpy.float32(centerImg.reshape(height * width,3))

        #find K dominant colors of the extracted area, to determine the color of the pathway
        compactness,labels,center = cv2.kmeans(K=10,
                                            flags = cv2.KMEANS_RANDOM_CENTERS,
                                            attempts=10,
                                            bestLabels=None,
                                            data=centerIm,
                                            criteria= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 3, 1.0))


        #convert dominant colors array from float32 [K,3] to uint8t [K,3]
        self.center = numpy.uint8(center)
        
    def detectPath(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_center = numpy.empty([numpy.size(self.center,0), numpy.size(self.center,1)]) # [K,3] matrix of K dominant colors in HSV
        mask_array = [] #array of K masks, one for each dominant color
        offset_lower = [15, 10, 20] # [H,S,V] negative offset
        offset_upper = [15, 10, 20] # [H,S,V] positive offset
        for i in range(numpy.size(self.center,0)):
            hsv_center[i,:] = cv2.cvtColor(numpy.array([[self.center[i,:]]]), cv2.COLOR_BGR2HSV).flatten() #convert color space of corresponding dominant color from BGR to HSV
            thresh_lower = numpy.subtract(hsv_center[i,:],offset_lower) #lower [H,S,V] boundary of accepted pathway color
            thresh_upper = numpy.add(hsv_center[i,:], offset_upper) #upper [H,S,V] boundary of accepted pathway color
            mask_array.append(cv2.inRange(hsv, thresh_lower, thresh_upper)) #create mask based on accepted color range of corresponding dominant color
        
        #create combined mask from K masks
        mask_comb = mask_array[0]
        for i in range(1,numpy.size(self.center,0)):
            mask_comb = cv2.bitwise_or(mask_comb,mask_array[i])

        #filtrate created mask
        mask_filter = cv2.medianBlur(mask_comb,5)
        mask_filter2 = cv2.medianBlur(mask_filter, 19)

        #find pathway border
        #contours, hierarchy = cv2.findContours(mask_filter2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        #img = cv2.drawContours(img, contours, -1, (0,255,0), 3)

        #calculate centre of the sidewalk
        ROWS = img.shape[0]
        COLS = img.shape[1]
        N = 3
        dN = int(ROWS/(2*N))
        cX = numpy.zeros(N + 1, dtype=int)
        cY = numpy.zeros(N + 1, dtype=int)
        
        cX[0] = int(COLS/2)
        cY[0] = ROWS
        
        for n in range(N):
            M = cv2.moments(mask_comb[ROWS - (n + 1)*dN:ROWS - n*dN, :])
            cX[n + 1] = int(M["m10"] / M["m00"])
            cY[n + 1] = int(M["m01"] / M["m00"]) + ROWS - (n + 1)*dN
        
        for n in range(N):
            cv2.line(img, (cX[n], cY[n]), (cX[n + 1], cY[n + 1]), (0, 255, 0), 5) 

        self.calcActuatingSig(cX, cY)
        return img, mask_comb
        # #show processed image
        # cv2.imshow('img', img)
        # cv2.imshow('mask_comb', mask_comb)
        # cv2.imshow('mask_f2', mask_filter2)

        # #wait needed to avoid instant program termination
        # cv2.waitKey(0)
    
    def calcActuatingSig(self, cX, cY):
        dx = cX[-1] - cX[0]
        dy = cY[-1] - cY[0]
        ang = math.atan2(dy, dx)        
        v = 0.5
        w = math.pi/2 + ang
        self.wp = w
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def camera_callback(self,img):
        # transform data from camera to img
        frame = numpy.fromstring(img.data, numpy.uint8)
        frame = frame.reshape(img.height, img.width, 3)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # on first call initialize mask based on dominant colors
        if self.initialized == 0:
            self.getPathColor(frame)

        # show img with detected path and mask
        imgDet, mask_comb = self.detectPath(frame)
        cv2.imshow("mask", mask_comb)
        cv2.imshow("path", imgDet)
        cv2.waitKey(1)
        initialized = 1

def main():
    rospy.init_node('path_detection')
    pub = rospy.Publisher('/mrvk_diff_drive_controller/cmd_vel', Twist, queue_size = 10)
    pathDet = PathDetector(pub)    
    rospy.Subscriber("/camera/image_raw", Image, pathDet.camera_callback)    
    rospy.spin()

if __name__ == '__main__':
    main()
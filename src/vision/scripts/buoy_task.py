#!/usr/bin/env python

import cv2
import math
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server as DynServer
from srmauv_msgs.msg import  *
from srmauv_msgs.srv import *
import vision.cfg.buoysConfig as Config





lowThresh=np.array([0,0,0])
highThresh=np.array([0,0,0])

cv2.namedWindow('image')

class Buoys:
    bridge=None
    lowThresh=np.array([0,0,0])
    highThresh=np.array([0,0,0])
    screen={'width':640,'height':480}
    image=None
    circleParams = {'minRadius':13, 'maxRadius': 0 }
    houghParams = (74, 11)
    allCentroidList = []
    allAreaList = []
    allRadiusList = []
    
    minContourArea = 250
   
    # Keep track of the previous centroids for matching
    previousCentroid = (-1, -1)
    previousArea = 0
    
    
    def rosimg2cv(self,ros_img):
        try:
            frame=self.bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)
            
        return frame
    
            
        

    def __init__(self):
        self.dyn_reconfigure_server=DynServer(config,self.reconfigure)
        image= np.zeros((320,512,3), np.uint8)
        
    
    def reconfigure(self,config,level):
        rospy.loginfo('Reconfigure request !')
        self.lowThresh[0]=config['lowL']
        self.lowThresh[1]=config['lowU']
        self.lowThresh[2]=config['lowV']
        self.highThresh[0]=config['highL']
        self.highThresh[1]=config['highU']
        self.highThresh[2]=config['highV']
        
        
        
    


    
    
    def threshold(im):
    
        contours, hierachy = cv2.findContours(im, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(im, contours,-1, (0,255,0), 3)
        contours = filter(lambda c: cv2.contourArea(c) >minContourArea, contours)
        contours = sorted(contours, key=cv2.contourArea, reverse=True) # Sort by largest contour
        if len(contours) > 0:
            largestContour = contours[0]
            mu = cv2.moments(largestContour)
            muArea = mu['m00']
            centroidToBump = (int(mu['m10']/muArea), int(mu['m01']/muArea))
        
            cv2.circle(img,centroidToBump,2,(0,255,0),3)
            st=str(int(mu['m10']/muArea))+" "+str(int(mu['m01']/muArea))
            pub.publish(std_msgs.msg.String(st))        
            rectArea = muArea
            previousCentroid = centroidToBump
            previousArea = rectArea
    
        else:
        # Find hough circles 
            circles = cv2.HoughCircles(binImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=30, param1=houghParams[0],
                                   param2=houghParams[1],
                                   minRadius = circleParams['minRadius'],
                                   maxRadius = circleParams['maxRadius'])
               
                # Check if center of circles inside contours
            if contours is not None:
                for contour in contours:
                    
                    mu = cv2.moments(contour)
                    muArea = mu['m00']
                    centroid = (mu['m10']/muArea, mu['m01']/muArea)
                    if circles is not None:
                        for circle in circles[0,:,:]:
                             circleCentroid = (circle[0], circle[1])
                             if abs(math.hypot(centroid[0]-circlecentroid[0],centroid[1]-circlecentroid[1])) < circle[2]:
                                 fb = True
                                    # Find new centroid by averaging the centroid and circle centroid
                             newCentroid =(int(centroid[0]+circleCentroid[0])/2,
                                                  int(centroid[1]+circleCentroid[1])/2)
                             allCentroidList.append(newCentroid)
    
                             allAreaList.append(cv2.contourArea(contour))
                             allRadiusList.append(circle[2])
                             # Draw circles
                             cv2.circle(img, newCentroid, circle[2], (255, 255, 0), 2)
                             cv2.circle(img, newCentroid, 2, (255, 0, 255), 3)        
               
                 # Find the circle with the largest radius
                    if not len(allCentroidList) == 0:
                        maxIndex = allRadiusList.index(max(allRadiusList))
                        centroidToBump = allCentroidList[maxIndex]
                        rectArea = allAreaList[maxIndex]
                   
                        previousCentroid = centroidToBump
                        previousArea = rectArea
                    else:
                        centroidToBump = previousCentroid
                        rectArea = previousArea
    
    # Draw new centroid
        #cv2.circle(scratchImgCol,centroidToBump, 3, (0, 255, 255), 2)


# RYB Stands for red yellow blue


cv2.imshow('image',image)
k = cv2.waitKey(1) & 0xFF
'''
redParams = {
                # 'lo3': (0, 204, 102), 'hi3': (22, 255, 255),                  #Robosub comp course 5:45pm
                # 'lo3': (0, 204, 0), 'hi3': (8, 255, 255),       
                # 'lo3': (0, 204, 107), 'hi3': (22, 255, 255),      
                'lo3': (0, 130, 97), 'hi3': (22, 255, 255),
                # 'lo4': (149, 134, 0), 'hi4': (255, 255, 242), # Bottom dark colours
                 'dilate': (9, 9), 'erode': (5,5), 'open': (5,5)}

greenParams = {'lo': (24, 30, 50), 'hi': (111, 255, 255),
                   'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}

blueParams = {'lo': (17, 18, 2), 'hi': (20, 255, 255),
                  'dilate': (11,11), 'erode': (5,5), 'open': (3,3)}
'''
curCol = None
#lo=[]
#hi=[]



c=1
while(c==1):
    # Capture frame-by-frame
    ret, img = cap.read()
    cv2.imshow("bal",img)
    # Keep track of the previous centroids for matching
    previousCentroid = (-1, -1)
    previousArea = 0
    img=cv2.resize(img,(640,480))
    channels=cv2.split(img)
    channels[0] = cv2.equalizeHist(channels[0])
    channels[1] = cv2.equalizeHist(channels[1])
    #channels[2] = cv2.equalizeHist(channels[2])
    img = cv2.merge(channels, img)
    img=cv2.bilateralFilter(img, -1, 5, 0.1)
    
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    img=cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)
    hsvImg=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    luvImg=cv2.cvtColor(img,cv2.COLOR_BGR2LUV)

    #hsvImg = np.array(hsvImg, dtype=np.uint8)

    gauss = cv2.GaussianBlur(luvImg, ksize=(5,5), sigmaX=10)

    sum = cv2.addWeighted(luvImg, 1.5, gauss, -0.6, 0)
    enhancedImg = cv2.medianBlur(sum, 3)
    ch=cv2.split(enhancedImg)
    mask = cv2.inRange(ch[2],v2,v1)
    mask1=cv2.inRange(ch[1],h2,h1)
    mask2=cv2.inRange(ch[2],s2,s1)
    cv2.imshow("thresh",mask)
    cv2.imshow("thresh1",mask1)
    cv2.imshow("thresh2",mask2)
    #cv2.imshow("bal1",luvImg)
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    '''
    mask = cv2.inRange(luvImg,lo,hi)
    print lo
    print hi
    #res=cv2.bitwise_and(img,img,mask=mask)
    cv2.imshow("thresh",mask)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)
   
    kern2 = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    threshImg = cv2.dilate(mask, kern2, iterations=2)
    cv2.imshow("thresh1",threshImg)
    '''
    threshImg=mask
    binImg=threshImg
    scratchImg = binImg.copy()
    scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
    cir(threshImg)
    cir(mask1)
    cir(mask2)
    cv2.imshow("bal",img)
    #cv2.imshow("ba",scratchImgCol)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()


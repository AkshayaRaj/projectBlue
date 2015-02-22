#!/usr/bin/env python


import roslib; roslib.load_manifest('vision')



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

import vision.cfg.buoyConfig as Config

from unittest import signals

from OpenGL.raw.GL.SGIX import framezoom

import signal







lowThresh=np.array([0,0,0])

highThresh=np.array([0,0,0])



cv2.namedWindow('image')



class Line:

    bridge=None

    lowThresh=np.array([0,0,0])

    highThresh=np.array([0,0,0])

    screen={'width':320,'height':240}

    image=None

    threshOut=None

    circleParams = {'minRadius':13, 'maxRadius': 0 }

    houghParams = (74, 11)

    allCentroidList = []

    allAreaList = []

    allRadiusList = []

    blur=True

    minContourArea = 250

    previousArea=None

    previousCentroid=None



    # Keep track of the previous centroids for matching

    previousCentroid = None

    previousArea = None

    found=None



    def rosimg2cv(self,ros_img):

        try:

            frame=self.bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)

        except CvBridgeError as e:

            rospy.logerr(e)

            rospy.loginfo("CvBridge error")



        return frame









    def __init__(self):

        self.dyn_reconfigure_server=DynServer(Config,self.reconfigure)

        #image= np.zeros((320,512,3), np.uint8)



        self.imgData={'detected':False}

        self.bridge=CvBridge()

        self.camera_topic=rospy.get_param('~image', '/sedna/bottom/image_raw')

        self.image_filter_pub=rospy.Publisher("/vision/image_filter",Image)
        self.threshOut_pub=rospy.Publisher("/vision/lineFollower/threshOut",Image)

        self.register()

        self.previousCentroid=(-1,-1)

        self.previousArea=0

    	#self.found=False







    def reconfigure(self,config,level):

        rospy.loginfo('Reconfigure request !')
        self.lowThresh[0]=config['loL']
        self.lowThresh[1]=config['loU']
        self.lowThresh[2]=config['loV']
        self.highThresh[0]=config['hiL']
        self.highThresh[1]=config['hiU']
        self.highThresh[2]=config['hiV']
        self.minContourArea=config['minContourArea']
        self.blur=config['blur']
        print "configured"
        return config

    def circles(self,cv_image):

        cv_image=cv2.resize(cv_image,dsize=(self.screen['width'],self.screen['height']))
        #if self.blur:
        #    cv_image=cv2.GaussianBlur(cv_image,ksize=[5,5],sigmaX=0)
        #Added Code
        inB, inG, inR = cv2.split(cv_image)
        avgR = np.mean(inR)
        avgG = np.mean(inG)
        avgB = np.mean(inB)
        avgGray = np.mean((avgB, avgG, avgR))
        if avgB == 0:
            outB = inB
        else:
            outB = (avgGray/avgB)*inB
        if avgG == 0:
            outG = inG
        else:
            outG = (avgGray/avgG)*inG
        if avgR == 0:
            outR = inR
        else:
            outR = (avgGray/avgR)*inR

        maxRGB = (np.max(outR), np.max(outG), np.max(outB))

        factor = np.max(maxRGB)
        if factor > 1:
            outR = 255*outR/factor
            outG = 255*outG/factor
            outB = 255*outB/factor

        outImg = cv2.merge((np.uint8(outB), np.uint8(outG), np.uint8(outR)))
        channels=cv2.split(outImg)
        channels[0] = cv2.equalizeHist(channels[0])
        channels[1] = cv2.equalizeHist(channels[1])
        #channels[2] = cv2.equalizeHist(channels[2])

        img = cv2.merge(channels, cv_image)
        img=cv2.bilateralFilter(img, -1, 5, 0.1)
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        img=cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)
        hsvImg=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        luvImg=cv2.cvtColor(img,cv2.COLOR_BGR2LUV)
        gauss = cv2.GaussianBlur(luvImg, ksize=(5,5), sigmaX=10)
        sum = cv2.addWeighted(luvImg, 1.5, gauss, -0.6, 0)
        enhancedImg = cv2.medianBlur(sum, 3)
        ch=cv2.split(enhancedImg)

        mask = cv2.inRange(ch[2],self.highThresh[2],self.lowThresh[2])
        mask1=cv2.inRange(ch[0],self.highThresh[0],self.lowThresh[0])
        mask2=cv2.inRange(ch[2],self.highThresh[1],self.lowThresh[1])

	mas=mask.copy()
	mas1=mask1.copy()
	mas2=mask2.copy()

        #ADDED

        #self.cir(mas)
        self.cir(mas1)
        #self.cir(mas2)



       # cv2.imshow(mask)

        #cv2.imshow(mask1)

        #cv2.imshow(mask2)

        mask_out=cv2.cvtColor(mask1,cv2.COLOR_GRAY2BGR)
	#mask_out1=cv2.cvtColor(mas1,cv2.COLOR_GRAY2BGR)
        try:

            self.image_filter_pub.publish(self.bridge.cv2_to_imgmsg(mas1,encoding="bgr8"))

	    self.threshOut_pub.publish(self.bridge.cv2_to_imgmsg(mask_out,encoding="bgr8"))
        except CvBridgeError as e:

            rospy.logerr(e)



    # ADDED FUNCTION



    def cir(self,img):



        contours, hierachy = cv2.findContours(img, cv2.RETR_EXTERNAL,

                                              cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(img, contours,-1, (0,255,0), 3)

        contours = filter(lambda c: cv2.contourArea(c) >self.minContourArea, contours)

        contours = sorted(contours, key=cv2.contourArea, reverse=True) # Sort by largest contour

        if len(contours) > 0:

            largestContour = contours[0]

            mu = cv2.moments(largestContour)
            x,y,w,h = cv2.boundingRect(largestContour)
            rect = cv2.minAreaRect(largestContour)

            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            print "points",box[0]
            #cv2.drawContours(img,[box],0,(0,0,255),2)
            maxArea = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 10:
                    #Find the center using moments
                    mu = cv2.moments(contour, False)
                    centroidx = mu['m10'] / mu['m00']
                    centroidy = mu['m01'] / mu['m00']
                    maxArea = area

                    #rectData['centroid'] = (centroidx, centroidy)
                    #rectData['rect'] = cv2.minAreaRect(contour)
                    rect = cv2.minAreaRect(contour)
            if maxArea > 0:
                #rectData['detected'] = True
                points = np.array(cv2.cv.BoxPoints(rect))

                #Find the blackline heading
                edge1 = points[1] - points[0]
                edge2 = points[2] - points[1]

                print "points1",points[1]
                print "points0",points[0]
                print "edge1",edge1


                #Choose the vertical edge
                if cv2.norm(edge1) > cv2.norm(edge2):
                    edge1[1] = edge1[1] if edge1[1] != 0.0 else math.copysign(0.01, edge1[1])
                    #rectData['angle'] = math.degrees(math.atan(edge1[0]/edge1[1]))
                    angle = math.degrees(math.atan(edge1[0]/edge1[1]))
                else:
                    edge2[1] = edge2[1] if edge2[1] != 0.0 else math.copysign(0.01, edge2[1])
                    #rectData['angle'] = math.degrees(math.atan(edge2[0]/edge2[1]))
                    angle = math.degrees(math.atan(edge2[0]/edge2[1]))

                #Chose angle to turn if horizontal

                if angle == 90:
                    if centroidx > screen['width'] / 2:
                        angle = -90
                elif angle == -90:
                    if centroidx < screen['width'] / 2:
                        angle = 90

                #Testing
                centerx = int(centroidx)
                centery = int(centroidy)
                cv2.circle(img, (centerx, centery), 5, (0, 255, 0))

                for i in range(4):
                    pt1 = (int(points[i][0]), int(points[i][1]))
                    pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                    cv2.line(img, pt1, pt2, (255, 0, 0))

                cv2.putText(img, str(angle), (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))



    def register(self):



#        self.image_sub=rospy.Subscriber(self.camera_topic,Image,self.cameraCallback)

        self.image_sub=rospy.Subscriber(self.camera_topic,Image,self.cameraCallback)

        rospy.loginfo("Subscribed to front camera")

        rospy.loginfo(self.camera_topic)



    def unregister(self):

        self.image_sub.unregister()

        rospy.loginfo("Unregistered front camera")



    def cameraCallback(self,ros_image):

        #rospy.loginfo("in cam")

        #cv_image=self.rosimg2cv(ros_image)

       # self.circles(cv_image)

        try:

            frame=self.bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)

        except CvBridgeError as e:

            rospy.logerr(e)

            rospy.loginfo("CvBridge error")

        self.circles(frame)



if __name__=="__main__":

    rospy.init_node("line_detector")

    buoys=Line()

    rospy.spin()

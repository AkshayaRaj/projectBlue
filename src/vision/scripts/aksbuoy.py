#usr/bin/env python

#import roslib; roslib.load_manifest('vision')

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
        signal.signal(signal.SIGINT,self.userQuit)
        self.imgData={'detected':False}
        self.bridge=CvBridge()
        self.camera_topic=rospy.get_param('~image', '/sedna/camera/front/image_raw')
        self.image_filter_pub=rospy.Publisher("/Vision/image_filter",Image)
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
        
        return config
        
    def userQuit(self,signal,frame):
        rospy.loginfo("Buoy server is shutting down")
        
    


    
    
    def circles(self,cv_image):
        cv_image=cv2.resize(cv_image,dsize=(self.screen['width'],self.screen['height']))
        #if self.blur:
        #    cv_image=cv2.GaussianBlur(cv_image,ksize=[5,5],sigmaX=0)
        
        channels=cv2.split(cv_image)
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
        mask1=cv2.inRange(ch[1],self.highThresh[0],self.lowThresh[0])
        mask2=cv2.inRange(ch[2],self.highThresh[1],self.lowThresh[1])
        
       # cv2.imshow(mask)
        #cv2.imshow(mask1)
        #cv2.imshow(mask2)
        mask_out=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        try:
            self.image_filter_pub.publish(self.bridge.cv2_to_imgmsg(mask_out, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)


        
        
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
    rospy.init_node("buoy_detector")
    buoys=Buoys()
    rospy.spin()

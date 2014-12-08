'''
Created on Jul 18, 2013

@author: gohew
'''
import cv2 as cv2
import cv2.cv as cv
import numpy as np

from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt

class QHistogram(QLabel):
    '''
    classdocs
    '''
    
    windowColor = [(255,0,0),(0,255,0),(0,0,255)]
    thresColor = (100,50,200)
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 0, 'hueHigh':180,'valLow':0,'valHigh':255,'grayLow': 0, 'grayHigh': 255}
    
    def __init__(self):
        '''
        Constructor
        '''
        super(QHistogram, self).__init__()
        self.show()
        
    def setParams(self,parameters):
        self.params = parameters
    
    #Compute the Histogram for three channels. Takes in a three channel image array.
    def getTripleHist(self,image):
        hist_bins = 256
        window_height = 200
        ranges = [0,255]
        i = 0
        imgArray = cv2.split(image)
        histImg = np.zeros((600,256,3),dtype=np.uint8)
        cv2.line(histImg,(self.params['hueLow'],window_height*(1)),(self.params['hueLow'],window_height*(1) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['hueHigh'],window_height*(1)),(self.params['hueHigh'],window_height*(1) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['satLow'],window_height*(2)),(self.params['satLow'],window_height*(2) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['satHigh'],window_height*(2)),(self.params['satHigh'],window_height*(2) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['valLow'],window_height*(3)),(self.params['valLow'],window_height*(3) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['valHigh'],window_height*(3)),(self.params['valHigh'],window_height*(3) -window_height),self.thresColor)
        for channelImg in imgArray:
            channel = 0
            hist = cv2.calcHist([channelImg], [channel], None, [256], [0, 256])
            #hist, _ = np.histogram(channelImg, hist_bins, (0, 255))
            cv2.normalize(hist,hist,0,200,cv2.NORM_MINMAX)
            for h in range(hist_bins):
                binVal = int(hist[h])
                cv2.line(histImg, (h,window_height*(i+1)), (h, window_height*(i+1)-binVal), self.windowColor[i],thickness=1)
            i = i + 1
        return histImg
            
    def updateHist(self,image):
        hsv_image = cv2.cvtColor(image, cv2.cv.CV_BGR2HSV)
        image = self.getTripleHist(hsv_image)
        #convert numpy mat to pixmap image
        qimg = QImage(image.data,image.shape[1], image.shape[0], QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)
        self.setPixmap(qpm)
        
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, desired_encoding="bgr8")
        # Convert from old OpenCV image trackbarnameto Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype
    
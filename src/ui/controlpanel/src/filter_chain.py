'''
Created on Jul 18, 2013

@author: gohew
'''
import numpy as np
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt
from PyQt4 import QtCore, QtGui, QtSvg
from PyQt4.QtWebKit import QGraphicsWebView
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import cv2
from qrangeslider import QRangeSlider
from histogram import QHistogram
from thresholding import Thresholding
import dynamic_reconfigure.client


#from com.ui import QRangeSlider

class Vision_filter(QWidget):
    '''
    classdocs
    '''
    rate = 20
    label_arr = None
    qle_arr = None
    qle_arr2 = None
    layout_arr = None
    hist = None
    thresholder = None
    isFront = 0
    params = {'C':0,'block':0, 'satLow': 0, 'satHigh': 255, 'hueLow': 0, 'hueHigh':180,'valLow':0,'valHigh':255}
   
    #slider_arr = [QSlider]
    def __init__(self):
        '''
        Constructor
        '''
        super(Vision_filter, self).__init__()
        self.thresholder = Thresholding()
        self.bridge = CvBridge()
        self.initUI()
        
    def initUI(self):
        label_arr = [QLabel(),QLabel(),QLabel(),QLabel(),QLabel(),QLabel(),QLabel()]
        #slider_arr = [QRangeSlider(),QRangeSlider(),QRangeSlider(),QRangeSlider(),QRangeSlider(),QRangeSlider(),QRangeSlider()] 
        self.slider_arr = [None,None,None,None,None,None]
        self.qle_arr = [None,None,None,None,None,None]
        self.qle_arr2 = [None,None,None,None,None,None]
        layout_arr = [None,None,None,None,None,None]
        main_layout = QHBoxLayout()
        
        ###Channel Params Sliders Declaration
        channel_front_layout = QVBoxLayout()
        channel_front_box = QGroupBox("Threshold Parameters")
        label_arr[0],self.slider_arr[0],self.qle_arr[0],layout_arr[0] = self.make_slider_box(" Hue",0,180)
        label_arr[1],self.slider_arr[1],self.qle_arr[1],layout_arr[1] = self.make_slider_box("Sat",0,255)
        label_arr[2],self.slider_arr[2],self.qle_arr[2],layout_arr[2] = self.make_slider_box("Val",0,255)
        
        filters_box = QGroupBox("Filters")
        filters_layout = QVBoxLayout()
        filters_cb = QComboBox()
        channel_cb = QComboBox()
        filters_cb.addItem("Default")
        filters_cb.addItem("Adaptive Thresholding")
        filters_cb.addItem("Otsu's Thresholding")
        filters_cb.activated[int].connect(self.onFilterCB)
        block_l,self.block_qle,block_layout = self.make_data_box("Block Size: ")
        self.C_l,self.C_qle,C_layout = self.make_data_box("C constant: ")
        
        channel_l,channel_cb, channelcb_layout = self.make_cb_box("Channel Selection: ")
        channel_cb.addItem("Channel 1")
        channel_cb.addItem("Channel 2")
        channel_cb.addItem("Channel 3")

        filters_cb.activated[int].connect(self.onChannelCB)
        filters_layout.addWidget(filters_cb)
        filters_layout.addLayout(block_layout)
        filters_layout.addLayout(C_layout)
        filters_layout.addLayout(channelcb_layout)
        filters_box.setLayout(filters_layout)
        self.C_qle.textEdited.connect(self.onQleChanged_C)
        self.block_qle.textEdited.connect(self.onQleChanged_block)
        
        self.slider_arr[0].setStart(0)
        self.slider_arr[1].setStart(0)
        self.slider_arr[2].setStart(0)
        self.slider_arr[0].setEnd(180)
        self.slider_arr[1].setEnd(255)
        self.slider_arr[2].setEnd(255)

        self.createColor()
        self.video_layout = QHBoxLayout()
        self.video_filter_l = QLabel("<b>Vision Filter Chain</b>")
        self.video_filter = QLabel()
        
        lo_h, self.lo_h_box, layout_lo_h = self.make_data_box("loH: ")
        hi_h, self.hi_h_box, layout_hi_h = self.make_data_box("hiH: ")
        lo_s, self.lo_s_box, layout_lo_s = self.make_data_box("loS: ")
        hi_s, self.hi_s_box, layout_hi_s = self.make_data_box("hiS: ")
        lo_v, self.lo_v_box, layout_lo_v = self.make_data_box("loV: ")
        hi_v,  self.hi_v_box, layout_hi_v = self.make_data_box("hiV: ")
        self.changeParamsBtn = QPushButton("Change")
        self.changeParamsBtn.clicked.connect(self.changeParamBtnHandler)
        params_layout = QHBoxLayout()
        params_layout.addLayout(layout_lo_h)
        params_layout.addLayout(layout_hi_h)
        params_layout.addLayout(layout_lo_s)
        params_layout.addLayout(layout_hi_s)
        params_layout.addLayout(layout_lo_v)
        params_layout.addLayout(layout_hi_v)
        params_layout.addWidget(self.changeParamsBtn)

        compression_l, self.compression_box, compression_provider = self.make_data_box("Compression Ratio: ")
        self.compression_box.setText("40")
        self.compression_Btn = QPushButton("Set")
        self.compression_Btn.clicked.connect(self.compression_btn_handler)
        
        self.video_layout.addWidget(self.video_filter_l)
        self.video_layout.addStretch()
        self.video_layout.addLayout(compression_provider)
        self.video_layout.addWidget(self.compression_Btn)
        
        channel_front_layout.addWidget(self.view)
        channel_front_layout.addLayout(layout_arr[0])
        channel_front_layout.addLayout(layout_arr[1])
        channel_front_layout.addLayout(layout_arr[2])
        channel_front_layout.addLayout(params_layout)
        channel_front_layout.addWidget(filters_box)

        channel_front_layout.addLayout(self.video_layout)
        channel_front_layout.addWidget(self.video_filter)
        channel_front_layout.addStretch(1)
        channel_front_box.setLayout(channel_front_layout)
        
        channel_layout = QVBoxLayout()
        channel_layout.addWidget(channel_front_box)
        
        ### Camera Imagery Declaration
        video_layout = QVBoxLayout()
        self.video_l = QLabel("<b>Camera Selection</b>")
        self.video_cb = QComboBox()
        self.video_cb.addItem("Front Camera")
        self.video_cb.addItem("Bottom Camera")
        self.video_cb.activated[int].connect(self.onActivated)
        
        self.video_top = QLabel()
        self.video_thres = QLabel()
        video_top_l = QLabel("<b>Camera Imagery</b>")
        video_bot_l = QLabel("<b>Thresholded Imagery</b>")
        video_layout.addWidget(self.video_l)
        video_layout.addWidget(self.video_cb)
        video_layout.addWidget(video_top_l)
        video_layout.addWidget(self.video_top)
        video_layout.addWidget(video_bot_l)
        video_layout.addWidget(self.video_thres)
        
        ### Histogram initialization
        hist_layout = QVBoxLayout()
        self.hist = QHistogram()
        hist_l = QLabel("<b>Histogram</b>")
        self.hist.setParams(self.params)
        hist_layout.addWidget(hist_l)
        hist_layout.addWidget(self.hist)
        
        main_layout.addLayout(channel_layout)
        main_layout.addLayout(hist_layout)
        main_layout.addLayout(video_layout)
        self.initTimer(self.rate)
        self.setLayout(main_layout)
        self.show()
    
    ### Update Vision Filter top video
    def update_image_filterchain(self,image):
        if image.encoding == "8UC1":
            cvRGBImg_top = cv2.cvtColor(self.rosimg2cv(image), cv2.cv.CV_GRAY2RGB)
        else:
            cvRGBImg_top = self.rosimg2cv(image)
            cvRGBImg_top = cv2.cvtColor(cvRGBImg_top, cv2.COLOR_BGR2RGB)
            #cv2.cvtColor(self.rosimg2cv(image), cv2.cv.CV_BGR2RGB)
        qimg = QImage(cvRGBImg_top.data,cvRGBImg_top.shape[1], cvRGBImg_top.shape[0], QImage.Format_RGB888)
        
        qpm = QPixmap.fromImage(qimg)
        self.video_filter.setPixmap(qpm.scaledToHeight(300))
        
    ### Update Vision Filter top video
    def update_image_visual(self,image, isRosImg=False):
        image = image if isRosImg else self.rosimg2cv(image)
        cvRGBImg_top = image
        cvRGBImg_top = cv2.cvtColor(cvRGBImg_top, cv2.COLOR_BGR2RGB)
        #cv2.cvtColor(image, cv2.cv.CV_BGR2RGB)
        qimg = QImage(cvRGBImg_top.data,cvRGBImg_top.shape[1], cvRGBImg_top.shape[0], QImage.Format_RGB888)
        
        qpm = QPixmap.fromImage(qimg)
        self.hist.updateHist(image) #TODO: put this back
        self.video_top.setPixmap(qpm.scaledToHeight(250))

    ### Update Vision Filter filter video
    def update_image_filter(self,image):
        thres_image = self.thresholder.threshold_image(self.rosimg2cv(image))
        thres_image =  cv2.cvtColor(thres_image, cv2.cv.CV_GRAY2RGB)
        thres_qimg = QImage(thres_image.data,thres_image.shape[1], thres_image.shape[0], QImage.Format_RGB888)
        thres_qpm = QPixmap.fromImage(thres_qimg)
        self.video_thres.setPixmap(thres_qpm.scaledToHeight(250))
        
    def onQleChanged_C(qseself,text):
        self.thresholder.params["C"] = int(text)
    def onQleChanged_block(self,text):
        self.thresholder.params["block"] = int(text)
    def onChannelCB(self,index):
        self.thresholder.channel = index
        
    def onFilterCB(self,index):
        self.thresholder.mode = index
        if(index  == 2):
            self.thresholder.setParams(self.params)
        
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, desired_encoding="bgr8")
        # Convert from old OpenCV image trackbarnameto Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype 
    
    def createColor(self):
        self.view = QLabel()
        #self.view.setGeometry(10, 10, , 100)
        #use full ABSOLUTE path to the image, not relative
        
        #Add draw a colour scale
        # pixmap = QtGui.QPixmap(os.getcwd() + "/icons/huescale.png")
        pixmap = QtGui.QPixmap("huescale.png")
        self.view.setPixmap((pixmap).scaled(710, pixmap.height()))

    def initTimer(self,time):
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.updateParams)
        self.timer.start(1000.0 / time)
    
    def updateParams(self):
        min ,max = self.slider_arr[0].getRange()
        #self.qle_arr[0].setText("min: " + str(min) + " max: " + str(max))
        
        self.params['hueLow'] = min
        self.params['hueHigh'] = max
        
        min ,max = self.slider_arr[1].getRange()
        self.params['satLow'] = min
        self.params['satHigh'] = max
        
        #self.qle_arr[1].setText("min: " + str(min) + " max: " + str(max))
        
        min ,max = self.slider_arr[2].getRange()
        self.params['valLow'] = min
        self.params['valHigh'] = max
        self.hist.setParams(self.params)
        self.thresholder.setParams(self.params)
        
        #self.qle_arr[2].setText("min: " + str(min) + " max: " + str(max))
        
        # self.setParams(self.params['hueLow'], self.params['hueHigh'],
        #                 self.params['satLow'], self.params['satHigh'],
        #                 self.params['valLow'], self.params['valHigh'])

    def changeParamBtnHandler(self):
        if self.lo_h_box.text() == "":
            lo_h = self.params['hueLow']
        else:
            lo_h = int(self.lo_h_box.text())

        if self.hi_h_box.text() == "":
            hi_h = self.params['hueHigh']
        else:
            hi_h = int(self.hi_h_box.text())

        if self.lo_s_box.text() == "":
            lo_s = self.params['satLow']
        else:
            lo_s = int(self.lo_s_box.text())
        
        if self.hi_s_box.text() == "":
            hi_s = self.params['satHigh']
        else:
            hi_s = int(self.hi_s_box.text())
        
        if self.lo_v_box.text() == "":
            lo_v = self.params['valLow']
        else:
            lo_v = int(self.lo_v_box.text())
        
        if self.hi_v_box.text() == "":
            hi_v = self.params['valHigh']
        else:
            hi_v = int(self.hi_v_box.text())

        self.updateParamsMap(lo_h, hi_h, lo_s, hi_s, lo_v, hi_v)

        # Update sliders
        self.updateSlider()

    def updateSlider(self):
        self.slider_arr[0].setStart(self.params['hueLow'])
        self.slider_arr[1].setStart(self.params['satLow'])
        self.slider_arr[2].setStart(self.params['valLow'])
        self.slider_arr[0].setEnd(self.params['hueHigh'])
        self.slider_arr[1].setEnd(self.params['satHigh'])
        self.slider_arr[2].setEnd(self.params['valHigh'])

    def updateParamsMap(self, loH, hiH, loS, hiS, loV, hiV):
        self.params['hueLow'] = loH
        self.params['hueHigh'] = hiH
        self.params['satLow'] = loS
        self.params['satHigh'] = hiS
        self.params['valLow'] = loV
        self.params['valHigh'] = hiV

        self.hist.setParams(self.params)
        self.thresholder.setParams(self.params)

    def setParams(self, loH, hiH, loS, hiS, loV, hiV):
        self.lo_h_box.setText(str(loH))
        self.hi_h_box.setText(str(hiH))
        self.lo_s_box.setText(str(loS))
        self.hi_s_box.setText(str(hiS))
        self.lo_v_box.setText(str(loV))
        self.hi_v_box.setText(str(hiV))

    def onActivated(self,index):
        self.isFront = index
    
    def make_cb_box(self,name):
        label = QLabel(name)
        qse = QComboBox()
        layout = QHBoxLayout()
        layout.addWidget(label)
        layout.addWidget(qse)
        return (label,qse,layout)
    
    def make_slider_box(self, name,min,max):
        label = QLabel(name)
        qse = QRangeSlider()
        qse.setMax(max)
        qse.setMin(min)
        qle = QLabel("")
        #qle2 = QLineEdit()
        layout = QHBoxLayout()
        #qle.setEnabled(False)
        layout.addWidget(qse)
        layout.addWidget(qle)
        layout.addWidget(label)
        #layout.addWidget(qle2)
        #layout.addStretch(1)
        
        return (label,qse, qle,layout)
    
    def make_data_box(self, name):
        label = QLabel(name)
        qle = QLineEdit()
        layout = QHBoxLayout()
        layout.addWidget(label)
        layout.addWidget(qle)
        #layout.addStretch(1)
        
        return (label, qle,layout)
    
    def compression_btn_handler(self):
        vision_client = dynamic_reconfigure.client.Client('/Vision/image_filter/compressed')
                  
        params = {'jpeg_quality': 40}
        config = vision_client.update_configuration(params)
        rospy.loginfo("Set vision compression to 40%")
    
    
        
#!/usr/bin/env python
import roslib
import rospy
import os

from bbauv_msgs.srv import *
from bbauv_msgs.msg import *
from nav_msgs.msg import Odometry
#import pynotify
import actionlib
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, quaternion_about_axis
import dynamic_reconfigure.client

from bbauv_msgs.msg import openups_stats
from bbauv_msgs.msg import openups
from batt_msgs.msg import Battery
#from bbauv_msgs.msg import Battery

import math
from math import pi
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt
import Queue
import threading
import thread
import signal
import sys

import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image

from bbauv_msgs.msg._thruster import thruster
from std_msgs.msg._Float32 import Float32
from std_msgs.msg._Int8 import Int8
from filter_chain import Vision_filter

class AUV_gui(QMainWindow):
    isLeak = False
    main_frame = None
    compass = None
    heading_provider = None
    depth_thermo = None
    client = None
    movebase_client = None
    batt_not = False
    isSonar = 0
    yaw = 0
    depth = 0
    pos_x = 0
    pos_y = 0
    isAlert = [True,True,True,True]
    cvRGBImg_front = None
    cvRGBImg_rfront = None
    cvRGBImg_bot = None
    cvRGBImg_f_bot = None
    cvRGBImg_f_frt = None
    isArmed = False
    update_freq = 40
    vision_filter_frame = None
    filter_image = None
    q_orientation = Queue.Queue()
    q_depth = Queue.Queue()
    q_earth_pos = Queue.Queue()
    q_rel_pos = Queue.Queue()
    q_controller = Queue.Queue()
    q_hull_status = Queue.Queue()
    q_mani = Queue.Queue()
    q_mode = Queue.Queue()
    q_controller_feedback = Queue.Queue()
    q_thruster = Queue.Queue()
    q_openups1 = Queue.Queue()
    q_openups2 = Queue.Queue()
    q_temp = Queue.Queue()
    q_altitude = Queue.Queue()
    q_cputemp = Queue.Queue()
    q_image_bot = None
    q_sonar = None
    q_image_front = None
    q_image_rfront = None
    data = {'yaw': 0, 'pitch' : 0,'roll':0, 'depth': 0,'mode':0, 'attitude':0,
            'pressure':0,'forward_setpoint':0,'sidemove_setpoint':0,
            'heading_setpoint':0,'depth_setpoint':0,'altitude':0,'heading_error':0,'openups1':Battery(), 'openups2':Battery(),
            'forward_error':0,'sidemove_error':0,'temp':0,'depth_error':0,'goal_id':"None",'thrusters':thruster(),
            'hull_status':hull_status(),'status':-1,'earth_pos':Odometry(),'rel_pos':Odometry(),
            'manipulators':manipulator(), 'cputemp':cpu_temperature()}
    counter = 0

    #Initialise subscribers/publishers
    isSubscribed = True
    thruster_sub = None
    orientation_sub = None
    thruster_sub = None
    depth_sub = None
    orientation_sub = None
    position_sub = None
    controller_sub = None
    mani_pub = None
    mani_sub = None
    earth_sub = None
    feedback_sub = None
    hull_status_sub = None
    openups_sub1 = None
    openups_sub2 = None
    temp_sub = None
    altitude_sub = None
    mode_sub = None
    frontcam_sub = None
    botcam_sub = None
    filter_sub = None
    frontfilter_sub = None
    botfilter_sub = None
    battery_sub = None
    cputemp_sub = None

    def __init__(self, parent=None):
        super(AUV_gui, self).__init__(parent)
        self.testing = rospy.get_param("~testing", False)

        self.main_tab = QTabWidget()
        self.main_frame = QWidget()
        self.vision_filter_frame = Vision_filter()
        #self.navigation_frame = Navigation_Map()
        self.main_tab.addTab(self.main_frame, "Telemetry")
        self.main_tab.addTab(self.vision_filter_frame, "Vision Filter")
        #self.main_tab.addTab(self.navigation_frame, "Navigation")

        goalBox =  QGroupBox("Goal Setter")
        depth_l , self.depth_box, layout4 = self.make_data_box("Depth:       ")
        sidemove_l, self.sidemove_box,layout2 = self.make_data_box("Sidemove:")
        forward_l, self.forward_box,layout1 = self.make_data_box("Forward:   ")
        heading_l, self.heading_box,layout3 = self.make_data_box("Heading:   ")
        forward_vel_l, self.forward_vel_box, layout_forward_vel = self.make_data_box("Forward Vel:")
        sidemove_vel_l, self.sidemove_vel_box, layout_sidemove_vel = self.make_data_box("Sidemove Vel:")

        self.depth_rev = QPushButton("Reverse")
        #layout4.addWidget(self.depth_rev)
        self.heading_rev = QPushButton("Reverse")
        #layout3.addWidget(self.heading_rev)

        self.sidemove_rev = QPushButton("Reverse")
        self.forward_rev = QPushButton("Reverse")

        roll_chk, self.roll_chkbox, layout_roll = self.make_data_chkbox("Roll:   ", checked=True)
        pitch_chk, self.pitch_chkbox, layout_pitch = self.make_data_chkbox("Pitch: ", checked=True)

        rel_heading_chk, self.rel_heading_chkbox,layout5 =self.make_data_chkbox("Rel:    ")
        rel_depth_chk, self.rel_depth_chkbox,layout6 =self.make_data_chkbox("Rel:    ")

        fwd_vel_chk_l, self.fwd_vel_chk,layout_fwd_chk =self.make_data_chkbox("Fwd Vel: ")
        sm_vel_chk_l, self.sm_vel_chk, layout_sm_chk =self.make_data_chkbox("Sm Vel: ")

        goal_heading_layout = QHBoxLayout()
        goal_heading_layout.addLayout(layout3)
        #goal_heading_layout.addLayout(layout5)
        goal_heading_layout.addStretch()

        goal_depth_layout = QHBoxLayout()
        goal_depth_layout.addLayout(layout4)
        #goal_depth_layout.addLayout(layout6)
        goal_depth_layout.addStretch()

        goal_forward_layout = QHBoxLayout()
        goal_forward_layout.addLayout(layout1)
        #goal_forward_layout.addLayout(layout_roll)
        #goal_forward_layout.addWidget(self.forward_rev)
        goal_forward_layout.addStretch()

        goal_sidemove_layout = QHBoxLayout()
        goal_sidemove_layout.addLayout(layout2)
        #goal_sidemove_layout.addLayout(layout_pitch)
        #goal_sidemove_layout.addWidget(self.sidemove_rev)
        goal_sidemove_layout.addStretch()

        goal_forward_vel_layout = QHBoxLayout()
        goal_forward_vel_layout.addLayout(layout_forward_vel)

        goal_sidemove_vel_layout = QHBoxLayout()
        goal_sidemove_vel_layout.addLayout(layout_sidemove_vel)

        goal_layout = QVBoxLayout()
        goal_layout.addLayout(goal_forward_layout)
        goal_layout.addLayout(goal_sidemove_layout)
        goal_layout.addLayout(goal_heading_layout)
        goal_layout.addLayout(goal_depth_layout)

        goal_vel_layout = QVBoxLayout()
        goal_vel_layout.addLayout(goal_forward_vel_layout)
        goal_vel_layout.addLayout(goal_sidemove_vel_layout)
        goal_vel_layout.addStretch()

        checkLayout = QVBoxLayout()
        checkLayout.addLayout(layout_roll)
        checkLayout.addLayout(layout_pitch)
        checkLayout.addLayout(layout5)
        checkLayout.addLayout(layout6)

        checkVelLayout = QVBoxLayout()
        checkVelLayout.addLayout(layout_fwd_chk)
        checkVelLayout.addLayout(layout_sm_chk)
        checkVelLayout.addStretch()

        bigGoalLayout = QHBoxLayout()
        bigGoalLayout.addLayout(goal_layout)
        bigGoalLayout.addLayout(checkLayout)
        bigGoalLayout.addLayout(goal_vel_layout)
        bigGoalLayout.addLayout(checkVelLayout)

        self.sidemove_rev.clicked.connect(self.sidemove_revHandler)
        self.depth_rev.clicked.connect(self.depth_revHandler)
        self.forward_rev.clicked.connect(self.forward_revHandler)
        self.heading_rev.clicked.connect(self.heading_revHandler)

        # Buttons Layout
        okButton = QPushButton("&Start Goal")
        cancelButton = QPushButton("&End Goal")
        self.disablePIDButton = QPushButton("&Disable PID")
        hoverButton = QPushButton("&Hover")
        surfaceButton = QPushButton("S&urface")
        homeButton = QPushButton("Home Base")
        self.modeButton = QPushButton("Default")
        self.unsubscribeButton = QPushButton("Unsubscribe")
        self.calDepthButton = QPushButton("&Calibrate Depth")
        mode_l, self.l_mode,mode_layout = self.make_data_box("Loc Mode:")
        self.l_mode.setAlignment(Qt.AlignCenter)
        self.l_mode.setEnabled(False)

        self.armButton = QPushButton("NOT ARMED")
        fireButton = QPushButton("&Fire")
        self.check1 = QCheckBox("&Top Torpedo")
        self.check2 = QCheckBox("&Bottom Torpedo")
        self.check3 = QCheckBox("&Grabber")
        self.check4 = QCheckBox("D&ropper")
        self.check5 = QCheckBox("")
        self.check6 = QCheckBox("")
        self.check7 = QCheckBox("")

        self.ledSelector = QComboBox()
        self.leds_map = zip(xrange(1, 10),
                           ['red', 'orange', 'yellow', 'green',
                            'blue', 'indigo', 'violet', 'white',
                            'off'])
        for led_map in self.leds_map:
            self.ledSelector.addItem(led_map[1])
        self.ledSelector.setCurrentIndex(len(self.leds_map)-1)
        self.ledSelector.currentIndexChanged.connect(self.ledSelectorCb)

        mani_layout = QVBoxLayout()
        mani_layout.addWidget(self.check2)
        mani_layout.addWidget(self.check1)
        mani_layout.addWidget(self.check3)
        mani_layout.addWidget(self.check4)
        mani_layout.addWidget(self.check5)
        mani_layout.addWidget(self.check6)
        mani_layout.addWidget(self.check7)
        mani_layout.addWidget(self.armButton)
        mani_layout.addWidget(fireButton)
        mani_layout.addWidget(self.ledSelector)
        maniBox = QGroupBox("Manipulators Console")
        maniBox.setLayout(mani_layout)

        self.armButton.clicked.connect(self.armBtnHandler)
        fireButton.clicked.connect(self.fireBtnHandler)
        okButton.clicked.connect(self.startBtnHandler)
        cancelButton.clicked.connect(self.endBtnHandler)
        surfaceButton.clicked.connect(self.surfaceBtnHandler)
        hoverButton.clicked.connect(self.hoverBtnHandler)
        homeButton.clicked.connect(self.homeBtnHandler)
        self.modeButton.clicked.connect(self.modeBtnHandler)
        self.disablePIDButton.clicked.connect(self.disablePIDHandler)
        self.unsubscribeButton.clicked.connect(self.unsubscribeHandler)
        self.calDepthButton.clicked.connect(self.calDepthHandler)
        vbox = QVBoxLayout()
        #hbox.addStretch(1)
        vbox.addWidget(okButton)
        vbox.addWidget(cancelButton)
        vbox.addWidget(self.disablePIDButton)
        vbox2 = QVBoxLayout()
        #hbox.addStretch(1)
        vbox2.addWidget(hoverButton)
        vbox2.addWidget(surfaceButton)
        vbox2.addWidget(homeButton)

        vbox3 = QVBoxLayout()
        #hbox.addStretch(1)
        vbox3.addLayout(mode_layout)
        vbox3.addWidget(self.modeButton)
        vbox3.addWidget(self.unsubscribeButton)
        vbox3.addWidget(self.calDepthButton)

        Navigation = QLabel("<b>Navigation</b>")
        xpos_l , self.xpos_box, layout6 = self.make_data_box("x coord:")
        ypos_l, self.ypos_box,layout7 = self.make_data_box("y coord:")
        self.goToPos = QPushButton("Go!")
        self.goToPos.clicked.connect(self.goToPosHandler)
        self.resetEarth = QPushButton("Reset E&arth")
        self.resetEarth.clicked.connect(self.resetEarthHandler)
        layout8 = QHBoxLayout()
        layout8.addWidget(self.goToPos)
        #layout8.addStretch()
        layout8.addWidget(self.resetEarth)

        vbox4 = QVBoxLayout()
        vbox4.addWidget(Navigation)
        vbox4.addLayout(layout6)
        vbox4.addLayout(layout7)
        vbox4.addLayout(layout8)

        goal_gui_layout = QHBoxLayout()
        goal_gui_layout.addLayout(bigGoalLayout)

        goalBox_layout = QVBoxLayout()
        goalBox_layout.addLayout(goal_gui_layout)

        goalBtn_layout = QHBoxLayout()
        goalBtn_layout.addLayout(vbox)
        goalBtn_layout.addLayout(vbox2)
        goalBtn_layout.addLayout(vbox3)
        #goalBox_layout.addStretch(50)
        goalBtn_layout.addLayout(vbox4)
        goalBox_layout.addLayout(goalBtn_layout)

        #vbox.addStretch(1)
        #vbox2.addStretch(1)
        vbox3.addStretch(1)
        goalBox.setLayout(goalBox_layout)

        self.compass = Qwt.QwtCompass()
        self.compass.setGeometry(0,0,190,190)
        self.compass.setLineWidth(4)
        self.compass.setMode(Qwt.QwtCompass.RotateNeedle)
        rose = Qwt.QwtSimpleCompassRose(16, 2)
        rose.setWidth(0.15)
        self.compass.setRose(rose)
        self.compass.setNeedle(Qwt.QwtCompassMagnetNeedle(
                Qwt.QwtCompassMagnetNeedle.ThinStyle))
        self.compass.setValue(220.0)
        self.compass.setScale(36, 5, 0)

        self.heading_provider = Qwt.QwtCompass()
        self.heading_provider.setLineWidth(4)
        self.heading_provider.setMode(Qwt.QwtCompass.RotateNeedle)
        rose = Qwt.QwtSimpleCompassRose(16, 2)
        rose.setWidth(0.15)
        self.heading_provider.setRose(rose)
        self.heading_provider.setNeedle(Qwt.QwtCompassMagnetNeedle(
                Qwt.QwtCompassMagnetNeedle.ThinStyle))

        compass_l = QLabel("Current")
        heading_l = QLabel("User Goal")
        compass_l.setAlignment(Qt.AlignHCenter)
        heading_l.setAlignment(Qt.AlignHCenter)

        compass_layout = QHBoxLayout()
        current_layout = QVBoxLayout()
        current_layout.addWidget(self.compass)
        current_layout.addWidget(compass_l)
        #current_layout.addStretch(1)
        user_layout = QVBoxLayout()
        user_layout.addWidget(self.heading_provider)
        user_layout.addWidget(heading_l)
        #user_layout.addStretch(1)
        compass_layout.addLayout(current_layout)
        compass_layout.addLayout(user_layout)

        compass_box = QGroupBox("AUV Heading")
        compass_box.setLayout(compass_layout)
        goal_gui_layout.addWidget(compass_box)

        #Depth Scale
        self.depth_thermo = Qwt.QwtThermo()
        self.depth_thermo.setPipeWidth(6)
        self.depth_thermo.setRange(0, 5)
        self.depth_thermo.setFillColor(Qt.green)
        self.depth_thermo.setAlarmColor(Qt.red)

        #Start Status Bar
        self.status_text = QLabel('Action Server idle')
        self.statusBar().addWidget(self.status_text, 1)

        #Attitude Information GUI
        attitudeBox =  QGroupBox("Attitude Information")
        self.attitudePanel1 = QTextBrowser()
        self.attitudePanel1.setStyleSheet("QTextBrowser { background-color : black; color :white;}")
        self.attitudePanel2 = QTextBrowser()
        self.attitudePanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel3 = QTextBrowser()
        self.attitudePanel3.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel4 = QTextBrowser()
        self.attitudePanel4.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel5 = QTextBrowser()
        self.attitudePanel5.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        attitude_layout = QHBoxLayout()
        attitude_layout.addWidget(self.attitudePanel1)
        attitude_layout.addWidget(self.attitudePanel2)
        attitude_layout.addWidget(self.attitudePanel3)
        attitude_layout.addWidget(self.attitudePanel4)
        attitude_layout.addWidget(self.attitudePanel5)
        attitudeBox.setLayout(attitude_layout)

        #Setpoint information
        setpointBox = QGroupBox("Setpoint Information")
        self.setpointPanel1 = QTextBrowser()
        self.setpointPanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.setpointPanel2 = QTextBrowser()
        self.setpointPanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")

        setpoint_layout = QVBoxLayout()
        setpoint_layout.addWidget(self.setpointPanel1)
        setpoint_layout.addWidget(self.setpointPanel2)
        setpointBox.setLayout(setpoint_layout)

        #Sensor and Actuator Information
        saBox = QGroupBox("Sensor and Actuator Information")
        self.saPanel1 = QTextBrowser()
        self.saPanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.saPanel2 = QTextBrowser()
        self.saPanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.saPanel3 = QTextBrowser()
        self.saPanel3.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.saPanel4 = QTextBrowser()
        self.saPanel4.setStyleSheet("QTextBrowser { background-color : black; color :white; }")

        sa_layout = QHBoxLayout()
        sa_layout.addWidget(self.saPanel1)
        sa_layout.addWidget(self.saPanel2)
        sa_layout.addWidget(self.saPanel3)
        sa_layout.addWidget(self.saPanel4)
        saBox.setLayout(sa_layout)

        #OpenUPS Information
        oBox = QGroupBox("Battery and Temp Information")
        self.oPanel1 = QTextBrowser()
        self.oPanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.oPanel2 = QTextBrowser()
        self.oPanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.oPanel3 = QTextBrowser()
        self.oPanel3.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.oPanel4 = QTextBrowser()
        self.oPanel4.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.lPanel1 = QTextBrowser()
        self.lPanel1.setStyleSheet("QTextBrowser { background-color : black; color : white}")

        o_layout = QHBoxLayout()
        o_layout.addWidget(self.oPanel1)
        o_layout.addWidget(self.oPanel2)
        o_layout.addWidget(self.oPanel3)
        o_layout.addWidget(self.oPanel4)
        o_layout.addWidget(self.lPanel1)
        oBox.setLayout(o_layout)

        #Leak sensors Information
        #lBox = QGroupBox("Leak Sensors")
        #self.lPanel1 = QTextBrowser()
        #self.lPanel1.setStyleSheet("QTextBrowser { background-color : black; color : white}")
        #leak_layout = QVBoxLayout()
        #leak_layout.addWidget(self.lPanel1)
        #lBox.setLayout(leak_layout)

        #Bottom layout
        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(oBox)
        #bottom_layout.addWidget(lBox)

        display_layout = QVBoxLayout()
        display_layout.addWidget(attitudeBox)
        display_layout.addWidget(saBox)
        display_layout.addLayout(bottom_layout)
        #display_layout.addWidget(oBox)
        overall_display_layout = QHBoxLayout()
        overall_display_layout.addLayout(display_layout)
        overall_display_layout.addWidget(setpointBox)

        main_layout = QHBoxLayout()
        main_Hlayout = QHBoxLayout()
        main_Vlayout = QVBoxLayout()
        main_Hlayout.addWidget(goalBox)
        main_Hlayout.addWidget(maniBox)
        main_Vlayout.addLayout(main_Hlayout)
        main_Vlayout.addLayout(overall_display_layout)
        main_layout.addLayout(main_Vlayout)
        main_layout.addWidget(self.depth_thermo)

        video_layout = QVBoxLayout()
        self.video_top = QLabel()
        self.video_bot = QLabel()
        video_top_l = QLabel("<b>Front Camera</b>")
        #video_bot_l = QLabel("<b>Bottom Camera</b>")
        video_bot_l = QComboBox()
        video_bot_l.addItem("Bottom Camera")
        video_bot_l.addItem("Sonar Image")
        video_layout.addWidget(video_top_l)
        video_layout.addWidget(self.video_top)
        video_layout.addStretch()
        video_layout.addWidget(video_bot_l)
        video_layout.addWidget(self.video_bot)
        main_layout.addLayout(video_layout)
        video_bot_l.activated[int].connect(self.onVideoActivated)

        #main_layout.addLayout(compass_layout)
        self.main_frame.setLayout(main_layout)
        self.setGeometry(300, 300, 1090, 760)
        self.setWindowTitle('Bumblebee AUV Control Panel')
        self.setWindowIcon(QIcon(os.getcwd() + '/icons/field.png'))
        self.setCentralWidget(self.main_tab)
        self.heading_provider.valueChanged.connect(self.valueChanged)
        self.initImage()
        self.initAction()
        self.initSub()
        self.initService()
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.on_timer)
        self.timer.start(1000.0 / self.update_freq)

        #if not pynotify.init("Basics"):
        #     sys.exit(1)

        #n = pynotify.Notification("Welcome", "Welcome to Bumblebee AUV Systems Control Panel!")
        #if not n.show():
        #     print "Failed to send notification"

    def on_timer(self):
        yaw = None
        depth = None
        orientation = None
        controller_setpoints = None
        controller_feedback = None
        manipulators = None
        thrusters = None
        hull_statuses = None
        rel_pos = None
        earth_pos = None
        openups1 = None
        openups2 = None
        temp = None
        altitude = None
        image_bot = None
        sonar_bot = None
        image_front = None
        image_rfront = None
        f_image_bot = None
        f_image_front = None
        mode = None
        cputemp = None
        '''Catch if queue is Empty exceptions'''
        try:
            orientation = self.q_orientation.get(False,0)
            self.q_orientation = Queue.Queue()
        except Exception,e:
            pass
        try:
            temp = self.q_temp.get(False,0)
            self.q_temp = Queue.Queue()
        except Exception,e:
            pass
        try:
            altitude = self.q_altitude.get(False,0)
            self.q_altitude = Queue.Queue()
        except Exception,e:
            pass
        try:
            mode = self.q_mode.get(False,0)
            self.q_mode = Queue.Queue()
        except Exception,e:
            pass

        try:
            openups1 = self.q_openups1.get(False,0)
            self.q_openups1 = Queue.Queue()
        except Exception, e:
            pass
        try:
            openups2 = self.q_openups2.get(False, 0)
            self.q_openups2 = Queue.Queue()
        except Exception,e:
            pass
        try:
            depth = self.q_depth.get(False,0)
            self.q_depth = Queue.Queue()
        except Exception,e:
            pass
        try:
            controller_setpoints = self.q_controller.get(False,0)
            self.q_controller = Queue.Queue()
        except Exception,e:
            pass
        try:
            controller_feedback = self.q_controller_feedback.get(False, 0)
            self.q_controller_feedback = Queue.Queue()
        except Exception,e:
            pass
        try:
            thrusters = self.q_thruster.get(False,0)
            self.q_thruster = Queue.Queue()
        except Exception,e:
            pass
        try:
            manipulators = self.q_mani.get(False,0)
            self.q_mani = Queue.Queue()
        except Exception,e:
            pass
        try:
            hull_statuses = self.q_hull_status.get(False,0)
            self.q_hull_status = Queue.Queue()
        except Exception,e:
            pass
        try:
            rel_pos = self.q_rel_pos.get(False,0)
            self.q_rel_pos = Queue.Queue()
        except Exception,e:
            pass
        try:
            earth_pos = self.q_earth_pos.get(False,0)
            self.q_earth_pos = Queue.Queue()
        except Exception,e:
            pass
        try:
            image_bot = self.q_image_bot
        except Exception,e:
            pass
        try:
            sonar_bot = self.q_sonar
        except Exception, e:
            pass
        try:
            cputemp = self.q_cputemp.get(False,0)
        except Exception,e:
            pass

        '''If data in queue is available store it into data'''
        if temp!= None:
            self.data['temp'] = temp
        if altitude!= None:
            self.data['altitude'] = altitude.data
        if manipulators != None:
            self.data['manipulators'] = manipulators
        if orientation != None:
            self.data['pitch'] = orientation.pitch
            self.data['yaw'] = orientation.yaw
            self.data['roll'] = orientation.roll
        if cputemp != None:
            self.data['cputemp'] = cputemp
        if hull_statuses != None:
            self.data['hull_status'] = hull_statuses
        if rel_pos != None:
            self.data['rel_pos'] = rel_pos
        if earth_pos != None:
            self.data['earth_pos'] = earth_pos
            #self.navigation_frame.receiveData(self.data['earth_pos'].pose.pose.position.x, self.data['earth_pos'].pose.pose.position.y )
        if depth != None:
            self.data['depth'] = depth.depth
            self.data['pressure'] = depth.pressure
        if thrusters != None:
            self.data['thrusters'] = thrusters
        if openups1 != None:
            self.data['openups1'] = openups1
        if openups2 != None:
            self.data['openups2'] = openups2
        if mode != None:
            self.data['mode'] = mode.data
        if controller_setpoints != None:
            self.data['heading_setpoint'] = controller_setpoints.heading_setpoint
            self.data['depth_setpoint'] = controller_setpoints.depth_setpoint
            self.data['forward_setpoint'] = controller_setpoints.forward_setpoint
            self.data['sidemove_setpoint'] = controller_setpoints.sidemove_setpoint

        if controller_feedback != None:
            self.data['forward_error'] =controller_feedback.feedback.forward_error
            self.data['heading_error'] =controller_feedback.feedback.heading_error
            self.data['sidemove_error'] =controller_feedback.feedback.sidemove_error
            self.data['depth_error'] =controller_feedback.feedback.depth_error
            self.data['goal_id'] = controller_feedback.status.goal_id.id
            self.data['status'] = controller_feedback.status.status
        if self.q_image_front != None:
            self.update_video_front(self.q_image_front)
        if self.vision_filter_frame.isFront == 0 and self.q_image_front!=None:
            self.update_video_rfront(self.q_image_front)
        if self.isSonar == 0:
            if image_bot != None:
                self.update_video_bot(image_bot)
        else:
            if sonar_bot != None:
                self.update_video_bot(sonar_bot)
        if self.filter_image != None:
            self.vision_filter_frame.update_image_filterchain(self.filter_image)

        self.depth_thermo.setValue(round(self.data['depth'],2))
        self.compass.setValue(int(self.data['yaw']))

        if self.data['mode']== 0:
            self.l_mode.setText("Default")
        elif self.data['mode'] == 1:
            self.l_mode.setText("Forward")
        elif self.data['mode'] == 2:
            self.l_mode.setText("Sidemove")

        self.attitudePanel1.setText("<b>YAW: <br>" + str(round(self.data['yaw'],2)) +
                                    "<br> PIT: " + str(round(self.data['pitch'],2)) +
                                    "<br>RLL: "+ str(round(self.data['roll'],2)) + "</b>")

        self.attitudePanel2.setText("<b>DEP: "+ str(round(self.data['depth'],2)) +
                                    "<br>PRE: " + str(round(self.data['pressure']/1000,2)) +
                                    "<br>ATT: " + str(round(self.data['altitude'],2)) + "</b>")

        self.attitudePanel3.setText("<b>POSX: " + str(round(self.data['earth_pos'].pose.pose.position.x,2)) +
                                    "<br> POSY: " + str(round(self.data['earth_pos'].pose.pose.position.y,2)) + "</b>")

        self.attitudePanel4.setText("<b>RPOSX: " + str(round(self.data['rel_pos'].pose.pose.position.x,2)) +
                                    "<br>RPOSY: " + str(round(self.data['rel_pos'].pose.pose.position.y,2)) +
                                    "<br>RPOSZ : " + str(round(self.data['rel_pos'].pose.pose.position.z,2)) + "</b>")

        self.attitudePanel5.setText("<b>VELX: " + str(round(self.data['rel_pos'].twist.twist.linear.x,2)) +
                                    "<br> VELY: " + str(round(self.data['rel_pos'].twist.twist.linear.y,2)) +
                                    "<br>VELZ: " + str(round(self.data['rel_pos'].twist.twist.linear.z,2)) + "</b>")

        self.saPanel1.setText("<b>THR1: " + str(self.data['thrusters'].speed1) +
                              "<br> THR2: " + str(self.data['thrusters'].speed2) +
                              "<br> THR3: " + str(self.data['thrusters'].speed3) +
                              "<br> THR4: " + str(self.data['thrusters'].speed4) + "</b>")
        self.saPanel2.setText("<b>THR5: " + str(self.data['thrusters'].speed5) +
                              "<br> THR6: " + str(self.data['thrusters'].speed6) +
                              "<br> THR7: " + str(self.data['thrusters'].speed7) +
                              "<br> THR8: " + str(self.data['thrusters'].speed8) + "</b>")

        mani_name = ["","","","","","",""]
        if self.data['manipulators'].mani_data & 1:
            mani_name[0] = "NONE"
        else:
            mani_name[0] = "FIRED"

        if self.data['manipulators'].mani_data & 2:
            mani_name[1] = "NONE"
        else:
            mani_name[1] = "FIRED"

        if self.data['manipulators'].mani_data & 4:
            mani_name[2] = "CLOSED"
        else:
            mani_name[2] = "OPENED"

        if self.data['manipulators'].mani_data & 8:
            mani_name[3] = "OPENED"
        else:
            mani_name[3] = "CLOSED"

        if self.data['manipulators'].mani_data & 16:
            mani_name[4] = "CLOSED"
        else:
            mani_name[4] = "OPENED"

        if self.data['manipulators'].mani_data & 32:
            mani_name[5] = "TRUE"
        else:
            mani_name[5] = "FALSE"

        if self.data['manipulators'].mani_data & 64:
            mani_name[6] = "TRUE"
        else:
            mani_name[6] = "FALSE"

        self.saPanel3.setText("<b>Grabber: " + mani_name[2]+"</b>")

        battery_notification1 = ""
        battery_notification2 = ""
        if self.data['openups1'].battery_percentage < 22.5:
            battery_notification1 = "BATTERY 1 DYING!"
        if self.data['openups2'].battery_percentage < 22.5:
            battery_notification2 = "BATTERY 2 DYING!"
        self.saPanel4.setText(battery_notification1 +
                              "<b><br>" + battery_notification2 +
                              "</b>")
        if self.data['openups1'].battery_percentage < 15.0 or \
            self.data['openups2'].battery_percentage < 15.0 and \
            not self.batt_not:
            pass
            # n = pynotify.Notification("BATTERY DYING BATTERY DYING!!!")
            # if not n.show():
            #     print "Failed to send battery dying notification"
            self.batt_not = True
        elif self.data['openups1'].battery_percentage > 20.0 and \
            self.data['openups2'].battery_percentage > 20.0:
            self.batt_not = False


#         self.saPanel3.setText("<b>Bot Tor: " + mani_name[0] +
#                               "<br>Top Tor: " + mani_name[1] +
#                               "<br>Grabber: " + mani_name[2] +
#                               "<br>Dropper: " + mani_name[3] +
#                               "</b>")
#
#         self.saPanel4.setText("<b>: " + mani_name[4] +
#                               "<br>: " + mani_name[5] +
#                               "<br>: " + mani_name[6] +
#                               "</b>")

        if (self.data['hull_status'].WaterDetA or self.data['hull_status'].WaterDetB
            or self.data['hull_status'].WaterDetC) and not self.isLeak:
            #n = pynotify.Notification("Leak Alert", "Water ingression in vehicle detected.\n Recover Vehicle NOW!!")
            #if not n.show():
            #    print "Failed to send notification"
            self.isLeak = True
        else:
            self.isLeak = False

        self.oPanel1.setText("<b>VOLT1: " + str(round(self.data['openups1'].cell6,2)) +
                              "<br>CUR1: " + str(round(self.data['openups1'].current,3)) +
                              "<br>%: " + str(round(self.data['openups1'].battery_percentage,2)) +
                              "<br>USE: " + str(round(self.data['openups1'].used_mAh,2)) +
                              "</b>")

        self.oPanel2.setText("<b>VOLT2: " + str(round(self.data['openups2'].cell6,2)) +
                              "<br>CUR2: " + str(round(self.data['openups2'].current,3)) +
                              "<br>%: " + str(round(self.data['openups2'].battery_percentage,2)) +
                              "<br>USE: " + str(round(self.data['openups2'].used_mAh,2)) +
                              "</b>")

        self.lPanel1.setText("<b>HU LEAK1: " + str(self.data['hull_status'].WaterDetA) +
                             "<br>HU LEAK2: " + str(self.data['hull_status'].WaterDetB) +
                             "<br>HY LEAK: " + str(self.data['hull_status'].WaterDetC) +
                             "</b>")

        self.oPanel3.setText("<b>TMP0: " + str(round(self.data['temp'],2)) +
                              "<br> TMP1: " + str(round(self.data['hull_status'].Temp0,2)) +
                              "<br> CPU: " + str(round(self.data['cputemp'].cores_ave)) +
                              "</b>")

        self.oPanel4.setText("<b>HUM: " + str(round(self.data['hull_status'].Humidity,2)) +
                              "<br>INT PRE: " + str(round(self.data['hull_status'].Int_pressure, 2)) +
                              "</b>")

#         s = "".join([i for i in self.data['goal_id'] if not i.isdigit()])
#         goal_string = s
        goal_string = self.data['goal_id'].partition('-')

        self.setpointPanel1.setText("<b>HDG: " + str(round(self.data['heading_setpoint'],2)) +
                                    "<br> FWD: " + str(round(self.data['forward_setpoint'],2)) +
                                    "<br>SIDE: "+ str(round(self.data['sidemove_setpoint'],2)) +
                                    "<br>DEP: "+ str(round(self.data ['depth_setpoint'],2)) +
                                    "<br><br>ID: " + goal_string[0] +
                                    "</b>")


        self.setpointPanel2.setText("<b>ST: " + self.get_status(self.data['status']) +
                                    "<br>HDG ERR: " + str(round(self.data['heading_error'],2)) +
                                    "<br> FWD ERR: " + str(round(self.data['forward_error'],2)) +
                                    "<br>SIDE ERR: "+ str(round(self.data['sidemove_error'],2)) +
                                    "<br>DEP ERR: "+ str(round(self.data ['depth_error'],2)) + "</b>")

    def showDialog(self,ups):
        pass
       # #QMessageBox.about(self,"Battery Low",)
       # n = pynotify.Notification("Battery Low", "OpenUPS " + str(ups) + " is low on battery.\n Replace now!")
       # if not n.show():
       #     print "Failed to send notification"

    def initService(self):
        #rospy.wait_for_service('set_controller_srv')
        #rospy.loginfo("set_controller Service ready.")
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)

        #rospy.wait_for_service('locomotion_mode_srv')
        rospy.loginfo("Locomotion Mode Service ready.")
        self.locomotion_mode_request = rospy.ServiceProxy('locomotion_mode_srv',locomotion_mode)

    def initImage(self):
        self.bridge = CvBridge()
        self.frontcam_sub = rospy.Subscriber(rospy.get_param('~front',"/front_camera/camera/image_rect_color_opt"),Image, self.front_callback)
        self.botcam_sub = rospy.Subscriber(rospy.get_param('~bottom',"/bot_camera/camera/image_rect_color_opt"),Image, self.bottom_callback)
        self.filter_sub = rospy.Subscriber(rospy.get_param('~filter',"/Vision/image_filter_opt"),Image, self.filter_callback)
        self.sonar_sub = rospy.Subscriber(rospy.get_param('~sonar', "/sonar_image"), Image, self.sonar_callback)

    def unsubscribe(self):
        rospy.loginfo("Unsubscribe from PID")
        self.thruster_sub.unregister()
        self.depth_sub.unregister()
        self.orientation_sub.unregister()
        self.position_sub.unregister()
        self.controller_sub.unregister()
        #self.mani_pub.unregister()
        self.mani_sub.unregister()
        self.earth_sub.unregister()
        self.feedback_sub.unregister()
        self.hull_status_sub.unregister()
        self.openups_sub1.unregister()
        self.openups_sub2.unregister()
        #self.temp_sub.unregister()
        self.altitude_sub.unregister()
        self.mode_sub.unregister()

        self.frontcam_sub.unregister()
        self.botcam_sub.unregister()
        self.filter_sub.unregister()
        self.sonar_sub.unregister()
        self.cputemp_sub.unregister()

    def initSub(self):
        rospy.loginfo("Subscribe to PID")
        self.thruster_sub = rospy.Subscriber("/thruster_speed",thruster, self.thruster_callback)
        self.depth_sub = rospy.Subscriber("/depth", depth ,self.depth_callback)
        self.orientation_sub = rospy.Subscriber("/euler", compass_data ,self.orientation_callback)
        self.position_sub = rospy.Subscriber("/WH_DVL_data", Odometry ,self.rel_pos_callback)
        self.controller_sub = rospy.Subscriber("/controller_points",controller,self.controller_callback)
        self.mani_pub = rospy.Publisher("/manipulators",manipulator)
        self.mani_sub = rospy.Subscriber("/manipulators",manipulator,self.manipulators_callback)
        self.earth_sub = rospy.Subscriber("/earth_odom",Odometry,self.earth_pos_callback)
        self.feedback_sub = rospy.Subscriber("/LocomotionServer/feedback",ControllerActionFeedback,self.controller_feedback_callback)
        self.hull_status_sub = rospy.Subscriber("/hull_status", hull_status, self.hull_status_callback)
        self.openups_sub1 = rospy.Subscriber("/battery1_status", Battery, self.openups_callback1)
        self.openups_sub2 = rospy.Subscriber("/battery2_status", Battery, self.openups_callback2)
        #self.temp_sub = rospy.Subscriber("/AHRS8_Temp",Float32,self.temp_callback)
        self.altitude_sub =  rospy.Subscriber("/altitude",Float32,self.altitude_callback)
        self.mode_sub = rospy.Subscriber("/locomotion_mode",Int8,self.mode_callback)
        self.cputemp_sub = rospy.Subscriber("/CPU_TEMP", cpu_temperature, self.cpu_callback)

    def ledSelectorCb(self, index):
        ledPub = rospy.Publisher("/led_strips", Int8)
        data = Int8(self.leds_map[index][0])
        for i in range(5):
            ledPub.publish(data)

    def get_status(self,val):
        if val == -1:
            return "NONE"
        if val == 0:
            return "PENDING"
        if val == 1:
            return "ACTIVE"
        if val == 2:
            return "PREEMPTED"
        if val == 3:
            return "SUCCEEDED"
        if val == 4:
            return "ABORTED"
        if val == 5:
            return "REJECTED"
        if val == 6:
            return "PREEMPTING"
        if val == 7:
            return "RECALLING"
        if val == 8:
            return "RECALLED"
        if val == 9:
            return "LOST"

    def unsubscribeHandler(self):
        if self.isSubscribed:
            self.unsubscribeButton.setText("Subscribe&n")
            self.unsubscribe()
#             self.navigation_frame.unregisterSub()
        else:
            self.unsubscribeButton.setText("U&nsubscribe")
            self.initSub()
            self.initImage()
#             self.navigation_frame.initSub
        self.isSubscribed = not self.isSubscribed

    def calDepthHandler(self):
        self.status_text.setText("Calibrating depth...")
        params = {'depth_offset': 0}
        config = self.controller_client.update_configuration(params)
        rospy.sleep(1.0)

        params = {'depth_offset': self.data['depth']}
        config = self.controller_client.update_configuration(params)
        rospy.loginfo("Depth calibrated")
        self.status_text.setText("Depth calibrated!! :) ")

    def sidemove_revHandler(self):
        rev_sidemove = -1.0 * float(self.sidemove_box.text())
        self.sidemove_box.setText(str(rev_sidemove))

    def depth_revHandler(self):
        rev_depth = -1.0 * float(self.depth_box.text())
        self.depth_box.setText(str(rev_depth))

    def forward_revHandler(self):
        rev_forward = -1.0 * float(self.forward_box.text())
        self.forward_box.setText(str(rev_forward))

    def heading_revHandler(self):
        rev_heading = -1.0 * float(self.heading_box.text())
        self.heading_box.setText(str(rev_heading))

    def disablePIDHandler(self):
          resp = self.set_controller_request(False, False, False,
                                             False, False, False,
                                             False, False,
                                             False, False)

    def goToPosHandler(self):
        xpos = float(self.xpos_box.text())
        ypos = float(self.ypos_box.text())
        self.status_text.setText("Moving to position x: " + str(xpos) + " ,y: " + str(ypos))
        def callService(x_pos, y_pos):
            bbLock = threading.Lock()
            try:
                bbLock.acquire()
                handle = rospy.ServiceProxy('/navigate2D', navigate2d)
                handle(x=x_pos, y=y_pos)
            except Exception as e:
                rospy.logerr("Unable to move to position")
            finally:
                bbLock.release()

        thread.start_new_thread(callService, (xpos, ypos))

    def homeBtnHandler(self):
        self.status_text.setText("Going home.... (0,0")
        handle = rospy.ServiceProxy('/navigate2D', navigate2d)
        handle(x=0, y=0)
        rospy.loginfo("Moving to home base (0,0)")

    def resetEarthHandler(self):
        self.status_text.setText("Earth odom resetted zero_distance")
        params = {'zero_distance': True}
        config = self.dynamic_client.update_configuration(params)
        self.navigation_frame.clearGraph()

    def hoverBtnHandler(self):
        self.status_text.setText("Hovering...")
        roll = False
        pitch = False
        if self.roll_chkbox.checkState():
            roll = True
        if self.pitch_chkbox.checkState():
            pitch = True
        resp = self.set_controller_request(True, True, True,
                                           True, True, False,
                                           True, True,
                                           False, False)
        goal = ControllerGoal
        goal.depth_setpoint = self.data['depth']
        goal.sidemove_setpoint = 0
        goal.heading_setpoint = self.data['yaw']
        goal.forward_setpoint = 0
        self.client.send_goal(goal, self.done_cb)

    def surfaceBtnHandler(self):
        self.status_text.setText("Surfacing... *gasp*")
        roll = False
        pitch = False
#         if self.roll_chkbox.checkState():
#             roll = True
        if self.pitch_chkbox.checkState():
            pitch = True
#         resp = self.set_controller_request(True, True, True, True, pitch, roll, False, False)
        resp = self.set_controller_request(True, True, True, True,
                                           True, False, False, False,
                                           False, False)
        goal = ControllerGoal
        goal.depth_setpoint = 0
        goal.sidemove_setpoint = 0
        goal.heading_setpoint = self.data['yaw']
        goal.forward_setpoint = 0
        self.client.send_goal(goal, self.done_cb)

    def modeBtnHandler(self):
        if(self.counter == 0):
            self.modeButton.setText("Forward")
            #Enable Forward Mode
            resp = self.locomotion_mode_request(True,False)
            self.counter = self.counter + 1
        elif(self.counter == 1):
            resp = self.locomotion_mode_request(False,True)
            self.modeButton.setText("Sidemove")
            #Enable Sidemove Mode
            self.counter = self.counter + 1
        elif(self.counter == 2):
            resp = self.locomotion_mode_request(False,False)
            self.modeButton.setText("Default")
            #Enable Default Mode

            self.counter = 0

    def startBtnHandler(self):
        self.status_text.setText("Action Client executing goal...")
        roll = False
        pitch = False
        fwd_vel = False
        sm_vel = False
        if self.roll_chkbox.checkState():
            roll = True
        if self.pitch_chkbox.checkState():
            pitch = True
        if self.fwd_vel_chk.checkState():
            fwd_vel = True
        if self.sm_vel_chk.checkState() and not fwd_vel:
            sm_vel = True
        resp = self.set_controller_request(True, True, True, True, pitch, roll,
                                           fwd_vel, sm_vel, False,False)
        goal = ControllerGoal
        # Forward
        if self.forward_box.text() == "":
             self.forward_box.setText("0")
        try:
            goal.forward_setpoint = float(self.forward_box.text())
        except:
            forward_string = "" + [i for i in forward_box.text if i.isdigit()]

         #Sidemove
        if self.sidemove_box.text() == "":
            self.sidemove_box.setText("0")
        goal.sidemove_setpoint = float(self.sidemove_box.text())

         #Heading
        if self.heading_box.text() == "":
            self.heading_box.setText(str(0))
            self.rel_heading_chkbox.setChecked(True)
            goal.heading_setpoint = self.data['yaw']
        elif self.rel_heading_chkbox.checkState():
            goal.heading_setpoint = (self.data['yaw'] + float(self.heading_box.text())) % 360
        else:
            goal.heading_setpoint = float(self.heading_box.text())

         #Depth
        if self.depth_box.text() == "":
            self.depth_box.setText(str(0))
            self.rel_depth_chkbox.setChecked(True)
            goal.depth_setpoint = self.data['depth']
        elif self.rel_depth_chkbox.checkState():
            goal.depth_setpoint = self.data['depth'] + float(self.depth_box.text())
        else:
            goal.depth_setpoint = float(self.depth_box.text())

        # Forward Velocity
        if self.forward_vel_box.text() == "":
             self.forward_vel_box.setText("0")
        try:
            goal.forward_vel_setpoint = float(self.forward_vel_box.text())
        except:
            forward_vel_str = "" + [i for i in forward_vel_box.text if i.isdigit()]
            goal.forward_vel_setpoint = float(forward_vel_str)

        # Sidemove Velocity
        if self.sidemove_vel_box.text() == "":
             self.sidemove_vel_box.setText("0")
        try:
            goal.sidemove_vel_setpoint = float(self.sidemove_vel_box.text())
        except:
            sidemove_vel_str = "" + [i for i in sidemove_vel_box.text if i.isdigit()]
            goal.sidemove_vel_setpoint = float(sidemove_vel_str)

        self.client.send_goal(goal, self.done_cb)

    def fireBtnHandler(self):
        if(self.isArmed):
            _manipulator = manipulator()
            servo_state = 0

            if(self.check1.checkState()):
                servo_state |= 1
            if(self.check2.checkState()):
                servo_state |= 2
            if(self.check3.checkState()):
                servo_state |= 4
            if(self.check4.checkState()):
                servo_state |= 8
            if(self.check5.checkState()):
                servo_state |= 16
            if(self.check6.checkState()):
                servo_state |= 32
            if(self.check7.checkState()):
                servo_state |= 64

            _manipulator = servo_state

            self.mani_pub.publish(_manipulator)

    def armBtnHandler(self):
        if(self.isArmed):
            self.isArmed = False
            self.armButton.setText("NOT ARMED")
        else:
            self.armButton.setText("ARMED")
            self.isArmed = True

    def done_cb(self,status,result):
        self.status_text.setText("Action Client completed goal!")
        #resp = self.set_controller_request(False, False, False, False, False, True, False, False)

    def movebase_done_cb(self,status,result):
        self.status_text.setText("Move Base Client completed goal!")

    def endBtnHandler(self):
        self.client.cancel_all_goals()
        #self.movebase_client.cancel_all_goals()
        self.status_text.setText("Action Client ended goal.")
        #resp = self.set_controller_request(False, False, False, False, False, False, False, False)

    def initAction(self):
        self.client = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        rospy.loginfo("Waiting for Action Server to connect.")
        self.status_text.setText("Waiting for Action Server to connect.")
        #self.client.wait_for_server()
        rospy.loginfo("Action Server connected.")
        self.status_text.setText("Action Server connected.")
        #self.movebase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #self.movebase_client.wait_for_server()
        rospy.loginfo("Mission connected to MovebaseServer")
        if not self.testing:
            self.dynamic_client = dynamic_reconfigure.client.Client('/earth_odom')
            rospy.loginfo("Earth Odom dynamic reconfigure initialised")

        self.controller_client = dynamic_reconfigure.client.Client('/Controller')
        rospy.loginfo("Controller client connected")

    def valueChanged(self,value):
        self.heading_box.setText(str(value))

    def make_data_box(self, name):
        label = QLabel(name)
        qle = QLineEdit()
        layout = QHBoxLayout()
        #qle.setEnabled(False)
        layout.addWidget(label)
        layout.addWidget(qle)
        layout.addStretch(1)
        qle.setFrame(False)

        return (label, qle, layout)

    def make_data_chkbox(self, name, checked=False):
        label = QLabel(name)
        qle = QCheckBox()
        layout = QHBoxLayout()
        if checked:
            qle.setChecked(True)
        layout.addWidget(label)
        layout.addWidget(qle)
        layout.addStretch(1)

        return (label, qle, layout)

    def onVideoActivated(self, index):
        self.isSonar = index

    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, desired_encoding="bgr8")
        # Convert from old OpenCV image trackbarnameto Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype

    def update_video_front(self,image):
        #convert numpy mat to pixmap image
        cvBGRImg_front = self.drawReticle(self.rosimg2cv(image))
        #cv2.cvtColor(self.drawReticle(self.rosimg2cv(image)), cv2.cv.CV_BGR2RGB)
        bbLock = threading.Lock()
        try:
            bbLock.acquire()
            cvRGBImg_front = cv2.cvtColor(cvBGRImg_front, cv2.COLOR_BGR2RGB)

            midX = 640/2.0 - 50.0
            midY = 480/2.0 - 60.0
            maxDeltaX = 640*0.05
            maxDeltaY = 480*0.05
            self.aimingCentroid = (midX, midY)
            cv2.rectangle(cvRGBImg_front,
<<<<<<< Updated upstream
                          (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                          (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                          (0, 255, 255), 2)
=======
                          (int(midX-13.0-maxDeltaX), int(midY+10.0-maxDeltaY)),
                          (int(midX-13.0+maxDeltaX), int(midY+10.0+maxDeltaY)),
                          (0, 255, 255), 2)  
>>>>>>> Stashed changes

            cv2.rectangle(cvRGBImg_front,
                          (int(midX-10.0-maxDeltaX), int(midY+25.0-maxDeltaY)),
                          (int(midX-10.0+maxDeltaX), int(midY+25.0+maxDeltaY)),
                          (255, 255, 255), 2)  

            cv2.circle(cvRGBImg_front, (int(self.aimingCentroid[0]), int(self.aimingCentroid[1])),
                97, (255, 255, 0), 2)

            camX = 640/2.0
            camY = 480/2.0
            cv2.rectangle(cvRGBImg_front,
                          (int(camX-maxDeltaX), int(camY-maxDeltaY)),
                          (int(camX+maxDeltaX), int(camY+maxDeltaY)),
                          (255, 0, 0), 2)


            qimg = QImage(cvRGBImg_front.data,cvRGBImg_front.shape[1], cvRGBImg_front.shape[0], QImage.Format_RGB888)
        finally:
            bbLock.release()
        qpm = QPixmap.fromImage(qimg)
        self.video_top.setPixmap(qpm.scaledToHeight(250))

    def update_video_rfront(self,image):
        bbLock = threading.Lock()
#        try:
#            bbLock.acquire()
#            qimg = QImage(cvRGBImg_top.data,cvRGBImg_top.shape[1], cvRGBImg_top.shape[0], QImage.Format_RGB888)
#        finally:
#            bbLock.release()
        self.vision_filter_frame.update_image_visual(image)
        self.vision_filter_frame.update_image_filter(image)

    def update_video_bot(self,image):
        cvBGRImg_bot = self.rosimg2cv(image)
        if self.isSonar == 1:
            cvBGRImg_bot = cv2.resize(cvBGRImg_bot, (360, 250))
        cvRGBImg_bot = cv2.cvtColor(cvBGRImg_bot, cv2.COLOR_BGR2RGB)
        qimg = QImage(cvRGBImg_bot.data,cvRGBImg_bot.shape[1], cvRGBImg_bot.shape[0], QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)
        self.video_bot.setPixmap(qpm.scaledToHeight(250))

        if self.vision_filter_frame.isFront == 1:
            self.vision_filter_frame.update_image_visual(image)
            self.vision_filter_frame.update_image_filter(image)

    def front_rcallback(self,image):
        try:
            self.q_image_rfront = image
        except CvBridgeError, e:
            print e

    def front_callback(self,image):
        try:
            self.q_image_front = image
        except CvBridgeError, e:
            print e

    def bottom_callback(self,image):
        try:
            self.q_image_bot = image
        except CvBridgeError, e:
            print e

    def sonar_callback(self, image):
        try:
            self.q_sonar = image
        except CvBridgeError, e:
            print e

    def mode_callback(self,mode):
        self.q_mode.put(mode)

    def thruster_callback(self,thruster):
        self.q_thruster.put(thruster)
    def orientation_callback(self,msg):
        self.q_temp.put(msg.temperature)
        self.q_orientation.put(msg)

    def cpu_callback(self, msg):
        self.q_cputemp.put(msg)

    def altitude_callback(self,altitude):
        self.q_altitude.put(altitude)

    def depth_callback(self,depth):
        self.q_depth.put(depth)

    def controller_callback(self,controller):
        self.q_controller.put(controller)

    def rel_pos_callback(self,pos):
        self.q_rel_pos.put(pos)

    def earth_pos_callback(self,earth):
        self.q_earth_pos.put(earth)

    def filter_callback(self,image):
        self.filter_image = image
    def hull_status_callback(self,hull):
        self.q_hull_status.put(hull)

    def controller_feedback_callback(self,feedback):
        self.q_controller_feedback.put(feedback)

    def openups_callback1(self,stats):
        self.q_openups1.put(stats)

    def openups_callback2(self,stats):
        self.q_openups2.put(stats)

    def temp_callback(self,temp):
        self.q_temp.put(temp)

    def manipulators_callback(self,mani):
        self.q_mani.put(mani)

    def drawReticle(self, origimg):
        yaw, pitch, roll = self.data['yaw'], self.data['pitch'], self.data['roll']

        DEGREE_PIXEL_RATIO = 0.1
        H_DEGREE_PIXEL_RATIO = 0.3
        height, width, _ = origimg.shape
        colour = (25, 25, 112)
        pitch_start, pitch_end = 40, height-40
        yaw_start, yaw_end = 40, width-40

        img = origimg

        mid_x, mid_y = width/2, height/2

        # Draw indicators
        cv2.line(img, (mid_x-70, mid_y), (mid_x-50, mid_y), (25, 25, 112), 2)
        cv2.line(img, (mid_x+50, mid_y), (mid_x+70, mid_y), (0, 0, 255), 2)
        cv2.line(img, (mid_x, 33), (mid_x-5, 38), (0, 0, 255), 2)
        cv2.line(img, (mid_x, 33), (mid_x+5, 38), (0, 0, 255), 2)
        cv2.line(img, (mid_x, pitch_end+13), (mid_x-5, pitch_end+10), (0, 0, 255), 2)
        cv2.line(img, (mid_x, pitch_end+13), (mid_x+5, pitch_end+10), (0, 0, 255), 2)

        # Multiply by 10 to work in integers
        origin_pitch = int(10 * (DEGREE_PIXEL_RATIO * (mid_y-pitch_start) + pitch))
        # Round to multiple of 25 lower than this
        BASE = 25
        closest_pitch = int(BASE * round(float(origin_pitch)/BASE))
        closest_pitch -= BASE if closest_pitch > origin_pitch else 0

        pitch_y = pitch_start + int((origin_pitch - closest_pitch) / (10 * DEGREE_PIXEL_RATIO))
        pitch_inc = int(BASE / (10 * DEGREE_PIXEL_RATIO))
        current_pitch = closest_pitch

        # Draw horizontal lines
        while pitch_y < pitch_end:
            thickness = 1
            offset = 6
            if current_pitch % 50 == 0:
                offset = 10
            if current_pitch % 100 == 0:
                offset = 18

            pt1 = (mid_x-offset, pitch_y)
            pt2 = (mid_x+offset, pitch_y)
            cv2.line(img, pt1, pt2, colour, thickness)

            if current_pitch % 100 == 0:
                txt = str(abs(current_pitch)/10)
                (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 2)
                pt = (mid_x-offset-txt_w-2, pitch_y + txt_h/2)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour, 2)

                pt = (mid_x+offset+2, pitch_y + txt_h/2)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour, 2)

            current_pitch -= BASE
            pitch_y += pitch_inc

        # Draw arc
        angle = int(180 - roll)
        cv2.ellipse(img, (mid_x, 140), (180, 120), angle, 75, 105, colour)
        arcpts = cv2.ellipse2Poly((mid_x, 140), (180, 120), angle, 75, 105, 15)
        for i, pt in enumerate(arcpts):
            disp_angle = (i-1) * 15
            txt = str(abs(disp_angle))
            txt_angle = np.deg2rad(-roll - 90 + disp_angle)
            (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 2)
            txt_x = int(pt[0] + 6 * math.cos(txt_angle)) - txt_w/2
            txt_y = int(pt[1] + 6 * math.sin(txt_angle))

            cv2.putText(img, txt, (txt_x, txt_y), cv2.FONT_HERSHEY_PLAIN, 0.8, colour, 2)
            cv2.ellipse(img, (pt[0], pt[1]), (1,1), 0, 0, 360, colour)

        # Draw horizontal band
        CARDINALS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']

        origin_yaw = int(-H_DEGREE_PIXEL_RATIO * (mid_x-yaw_start) + yaw)
        # Round to multiple of 5 greater than this
        H_BASE = 5
        closest_yaw = int(H_BASE * round(float(origin_yaw)/H_BASE))
        closest_yaw += H_BASE if closest_yaw < origin_yaw else 0

        yaw_x = 5 + yaw_start + int((closest_yaw - origin_yaw) / float(H_DEGREE_PIXEL_RATIO))
        yaw_inc = int(H_BASE / float(H_DEGREE_PIXEL_RATIO))
        current_yaw = closest_yaw

        yaw_bottom = pitch_end + 30

        while yaw_x < yaw_end:
            thickness = 1
            offset = 3 if current_yaw % 15 else 6

            pt1 = (yaw_x, yaw_bottom)
            pt2 = (yaw_x, yaw_bottom - offset)
            cv2.line(img, pt1, pt2, colour, thickness)

            if current_yaw % 15 == 0:
                disp_yaw = current_yaw if current_yaw >= 0 else current_yaw + 360
                disp_yaw = disp_yaw if current_yaw < 360 else current_yaw - 360
                txt = str(disp_yaw) if current_yaw % 45 else CARDINALS[disp_yaw / 45]
                (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 2)
                pt = (yaw_x-txt_w/2, yaw_bottom - txt_h)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour, 2)

            current_yaw += H_BASE
            yaw_x += yaw_inc

        return img

    def signal_handler(self, signal, frame):
        sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('Control_Panel', anonymous=True)
    app = QApplication(sys.argv)
    form = AUV_gui()
    signal.signal(signal.SIGINT, form.signal_handler)
    form.show()
    app.exec_()



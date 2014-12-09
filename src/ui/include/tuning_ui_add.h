/* 
	controlui_add.cpp
	Header file for Control UI 
	Date created: 9 Jan 2014
	Author: Lynnette
*/

#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <QLineEdit>
#include <QVector>
#include <QTimer>
#include <QObject>

#include "../src/tuning_ui.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <srmauv_msgs/ControllerAction.h>
#include <srmauv_msgs/thruster.h>
#include <srmauv_msgs/controller.h>
#include <srmauv_msgs/ControlData.h>
#include <srmauv_msgs/ControllerAction.h>
#include <srmauv_msgs/ControllerGoal.h>
#include <srmauv_msgs/set_controller.h>
#include <srmauv_msgs/pid_info.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/ReconfigureResponse.h>

#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <iomanip>
#include <map>
#include <utility>

	
#include <qstring.h>
#include <qtimer.h>
#include <qvector.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string/classification.hpp>


using namespace std;

//Utilities
string getdate();

//Qt UI callbacks
void updateStatus();
void updateGraph();
void updateIncomingParams();
void saveFile();
void openTheFile();
void fire();
void enableButton();
void sendButton();
void tuneButton();
void dofSelected(int index);
void graphTypeChanged(const QString& type);
void mouseMoved(QMouseEvent*);

#include "tuning_ui.h"
#include "tuning_ui_add.h"
#include <ros/ros.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <termios.h>
#include <signal.h>

#include "tuning_ui.h"

//Convenient stuffs for dynamic reconfiguring
#define numParams 30
string dynamicParams[31] =
{
		"depth_Kp", "depth_Ti", "depth_Td", "depth_min", "depth_max",
		"pitch_Kp", "pitch_Ti",	"pitch_Td", "pitch_min", "pitch_max",
		"roll_Kp", "roll_Ti", "roll_Td", "roll_min", "roll_max",
		"heading_Kp", "heading_Ti", "heading_Td", "heading_min", "heading_max",
		"forward_Kp", "forward_Ti", "forward_Td", "forward_min", "forward_max",
		"sidemove_Kp", "sidemove_Ti", "sidemove_Td", "sidemove_min", "sidemove_max"
};

//index for each DOF in the dynamicParams array
int depthIndex = 0, pitchIndex = 5, rollIndex = 10,
		headingIndex = 15, forwardIndex = 20, sidemoveIndex = 25;

//Types for each DOF params
string paramsTypes[] = {"double_t", "double_t", "double_t", "int_t", "int_t"};

std::string roundString(double val, int precision) {
	std::ostringstream ss;
	ss << std::fixed << std::setprecision(precision);
	ss << val;
	std::string s = ss.str();

	return s;
}

class ControlUI {
private:
	ros::NodeHandle private_nh;

	void initialiseDefault();
	void initializeGraph();
public:
	ControlUI();

	ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber thruster_sub;
	ros::Subscriber errorSub;
	ros::Subscriber graph_update;
	ros::Subscriber pidInfoSub;

	Ui::ControlSysUI ui;
	QMainWindow *window;
	ros::Time startTime;

	bool live;
	bool enable;

	map<string, string> params; //Map for dynamic parameters

	double error;

	//For graph
	string graphType;
	int startIndex, endIndex;
	double graphOut, graphSetPt;
	double x_org;

	int thruster1, thruster2, thruster3, thruster4,
	thruster5, thruster6, thruster7, thruster8;

	double total, p, i, d;

	void initialiseParameters();
	void autoSave();
	void loadControlParams();

	void subscribeToData();

	//Data callbacks
	void thruster_val_callback(const srmauv_msgs::thruster::ConstPtr& msg);
	void controllerPointsCallBack(const srmauv_msgs::controller::ConstPtr& data);
	void errorCallBack(const srmauv_msgs::ControllerActionFeedbackConstPtr& feedback);
	void pidInfoCallback(const srmauv_msgs::pid_infoConstPtr& pidInfo);

	//Helper functions to update Dynamic Reconfigure params
	void updateParameter(string paramName, bool val, dynamic_reconfigure::Config&);
	void updateParameter(string paramName, double val, dynamic_reconfigure::Config&);
	void updateParameter(string paramName, int val, dynamic_reconfigure::Config&);
	void updatePIDChecks(bool depth, bool heading, bool forward, bool sidemove, bool roll, bool pitch);
	void loadPIDChecks();
	void loadDynamicParams();

	QLineEdit* configureWidgets[5];
};
ControlUI* controlUI;

ControlUI::ControlUI() : nh(), private_nh("~"), live(true), enable(true) {
	startTime = ros::Time::now();
	private_nh.param("live", live, bool(true));

	window = new QMainWindow;
	ui.setupUi(window);
	window->setFixedSize(window->geometry().width(), window->geometry().height());
	configureWidgets[0] = ui.conKpVal;
	configureWidgets[1] = ui.conTiVal;
	configureWidgets[2] = ui.conTdVal;
	configureWidgets[3] = ui.conMinVal;
	configureWidgets[4] = ui.conMaxVal;

	if (!live){
		ROS_INFO("%s", "Not going live");
	} else {
		ROS_INFO("%s", "Going live!");
		subscribeToData();
	}

	graphOut = 0.0;
	graphSetPt = 0.0;
	graphType = "Depth";

	startIndex = 0;
	endIndex = 5;

	initialiseDefault();
	initialiseParameters();
	initializeGraph();
}

void ControlUI::errorCallBack(const srmauv_msgs::ControllerActionFeedbackConstPtr& feedback){
	if (graphType == "Heading") {
		error = feedback->feedback.heading_error;
	} else if (graphType == "Forward") {
		error = feedback->feedback.forward_error;
	} else if (graphType == "Side") {
		error = feedback->feedback.sidemove_error;
	} else if (graphType == "Depth") {
		error = feedback->feedback.depth_error;
	} else {
		error = 0;
	}
}

void ControlUI::pidInfoCallback(const srmauv_msgs::pid_infoConstPtr& info) {
	if (graphType == "Depth") {
		p = info->depth.p;
		i = info->depth.i;
		d = info->depth.d;
		total = info->depth.total;
	} else if (graphType == "Heading") {
		p = info->heading.p;
		i = info->heading.i;
		d = info->heading.d;
		total = info->heading.total;
	} else if (graphType == "Forward") {
		p = info->forward.p;
		i = info->forward.i;
		d = info->forward.d;
		total = info->forward.total;
	} else if (graphType == "Side") {
		p = info->sidemove.p;
		i = info->sidemove.i;
		d = info->sidemove.d;
		total = info->sidemove.total;
	} else if (graphType == "Roll") {
		p = info->roll.p;
		i = info->roll.i;
		d = info->roll.d;
		total = info->roll.total;
	} else if (graphType == "Pitch") {
		p = info->pitch.p;
		i = info->pitch.i;
		d = info->pitch.d;
		total = info->pitch.total;
	}
}

void ControlUI::subscribeToData() {
	//Thrusters Subscriber [thrusters.msg]
	thruster_sub = nh.subscribe("/thruster_speed", 1, &ControlUI::thruster_val_callback, this);

	errorSub = nh.subscribe("/LocomotionServer/feedback", 1, &ControlUI::errorCallBack, this);

	//Graph controller points
	graph_update = nh.subscribe("/controller_targets", 1, &ControlUI::controllerPointsCallBack, this);

	pidInfoSub = nh.subscribe("/pid_info", 1, &ControlUI::pidInfoCallback, this);
}

//Helper functions to update Dynamic Reconfigure params
void ControlUI::updateParameter(string paramName, bool val, dynamic_reconfigure::Config& conf) {
	dynamic_reconfigure::BoolParameter bool_param;

	bool_param.name = paramName;
	bool_param.value = val;
	conf.bools.push_back(bool_param);
}

void ControlUI::updateParameter(string paramName, double val, dynamic_reconfigure::Config& conf) {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;

	double_param.name = paramName;
	double_param.value = val;
	conf.doubles.push_back(double_param);
}

void ControlUI::updateParameter(string paramName, int val, dynamic_reconfigure::Config& conf) {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::IntParameter integer_param;

	integer_param.name = paramName;
	integer_param.value = val;
	conf.ints.push_back(integer_param);
}

void ControlUI::updatePIDChecks(bool depth, bool heading, bool forward, bool sidemove, bool roll, bool pitch) {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::Config conf;

	updateParameter("depth_PID", depth, conf);
	updateParameter("heading_PID", heading, conf);
	updateParameter("forward_PID", forward, conf);
	updateParameter("sidemove_PID", sidemove, conf);
	updateParameter("roll_PID", roll, conf);
	updateParameter("pitch_PID", pitch, conf);

	srv_req.config = conf;
	ros::service::call("/controller/set_parameters", srv_req, srv_resp);
	controlUI->ui.statusbar->showMessage("PID Checks sent!", 3000);
}

void ControlUI::loadDynamicParams() {
	for (int i = 0; i < numParams; i++) {
		if (paramsTypes[i%5] == "double_t") {
			double val;
			ros::param::param<double>("/controller/" + dynamicParams[i], val, 0.0);
			params[dynamicParams[i]] = boost::lexical_cast<string>(val);
		} else if (paramsTypes[i%5] == "int_t") {
			int val;
			ros::param::param<int>("/controller/" + dynamicParams[i], val, 0);
			params[dynamicParams[i]] = boost::lexical_cast<string>(val);
		}
	}
}

void ControlUI::loadPIDChecks() {
	bool forward, sidemove, heading, depth, pitch, roll;

	ros::param::param<bool>("/controller/forward_PID", forward, false);
	ros::param::param<bool>("/controller/sidemove_PID", sidemove, false);
	ros::param::param<bool>("/controller/heading_PID", heading, false);
	ros::param::param<bool>("/controller/depth_PID", depth, false);
	ros::param::param<bool>("/controller/pitch_PID", pitch, false);
	ros::param::param<bool>("/controller/roll_PID", roll, false);

	ui.roll_check->setChecked(roll);
	ui.pitch_check->setChecked(pitch);
	ui.fwd_check->setChecked(forward);
	ui.depth_check->setChecked(depth);
	ui.yaw_check->setChecked(heading);
	ui.sm_check->setChecked(sidemove);
}

void ControlUI::initialiseDefault() {
	params.clear();

	loadDynamicParams();
	loadPIDChecks();

	thruster1 = 0; thruster2 = 0; thruster3 = 0; thruster4 = 0;
	thruster5 = 0; thruster6 = 0; thruster7 = 0; thruster8 = 0;
}

void ControlUI::initialiseParameters() {
	//Controls part
	loadControlParams();
}

void ControlUI::autoSave() {
	FILE* output;
	output = fopen("controlParams.txt", "w");
	for (int i = 0; i < numParams; i++) {
		string curParam = dynamicParams[i];
		fprintf(output, (curParam + ": %s\n").c_str(), params[curParam].c_str());
	}
	fclose(output);
}

void ControlUI::controllerPointsCallBack(const srmauv_msgs::controller::ConstPtr& data) {
	if (graphType == "Depth") {
		graphSetPt = data->depth_setpoint;
		graphOut = data->depth_input;
	} else if (graphType == "Heading") {
		graphSetPt = data->heading_setpoint;
		graphOut = data->heading_input;
	} else if (graphType == "Forward") {
		graphSetPt = data->forward_setpoint;
		graphOut = data->forward_input;
	} else if (graphType == "Side") {
		graphSetPt = data->sidemove_setpoint;
		graphOut = data->sidemove_input;
	} else if (graphType == "Roll") {
		graphSetPt = data->roll_setpoint;
		graphOut = data->roll_input;
	} else if (graphType == "Pitch") {
		graphSetPt = data->pitch_setpoint;
		graphOut = data->pitch_input;
	}
}

//To plot the graph of sensors and setpt
void ControlUI::initializeGraph() {
	//Make legend visible
	//ui.graph_canvas->legend->setVisible(true);
	//ui.graph_canvas->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignRight|Qt::AlignBottom);

	//Add the graphs
	ui.graph_canvas->addGraph();
	ui.graph_canvas->graph(0)->setName("Setpt");
	ui.graph_canvas->graph(0)->setPen(QPen(Qt::red));

	ui.graph_canvas->addGraph();
	ui.graph_canvas->graph(1)->setName("Output");
	ui.graph_canvas->graph(1)->setPen(QPen(Qt::blue));

	// give the axes some labels:
	ui.graph_canvas->xAxis->setLabel("Time (s)");

	//For user interaction
	ui.graph_canvas->setInteraction(QCP::iRangeDrag, true);
	ui.graph_canvas->setInteraction(QCP::iRangeZoom, true);
	QObject::connect(ui.graph_canvas, &QCustomPlot::mouseMove, mouseMoved);

	x_org = 0;

	//Plot the graph
	ui.graph_canvas->replot();
}

void ControlUI::thruster_val_callback (const srmauv_msgs::thruster::ConstPtr& msg) {
	thruster1 = msg->speed1;
	thruster2 = msg->speed2;
	thruster3 = msg->speed3;
	thruster4 = msg->speed4;
	thruster5 = msg->speed5;
	thruster6 = msg->speed6;
	thruster7 = msg->speed7;
	thruster8 = msg->speed8;
}

void ControlUI::loadControlParams() {
	for (int i = startIndex; i < endIndex; i++) {
		int offsetIndex = i % 5;
		configureWidgets[offsetIndex]->setText(QString::fromStdString(params[dynamicParams[i]]));
	}
}

//////////////////////////
// Non Class functions //
/////////////////////////

void updateStatus() {
	controlUI->error = controlUI->graphOut - controlUI->graphSetPt;
	controlUI->ui.error_val->setText(QString::fromStdString(roundString(controlUI->error, 4)));
	controlUI->ui.setpt_val->setText(QString::fromStdString(roundString(controlUI->graphSetPt, 4)));
	controlUI->ui.sensor_val->setText(QString::fromStdString(roundString(controlUI->graphOut, 4)));

	controlUI->ui.thruster_val_1->setText(QString::number(controlUI->thruster1));
	controlUI->ui.thruster_val_2->setText(QString::number(controlUI->thruster2));
	controlUI->ui.thruster_val_3->setText(QString::number(controlUI->thruster3));
	controlUI->ui.thruster_val_4->setText(QString::number(controlUI->thruster4));
	controlUI->ui.thruster_val_5->setText(QString::number(controlUI->thruster5));
	controlUI->ui.thruster_val_6->setText(QString::number(controlUI->thruster6));
	controlUI->ui.thruster_val_7->setText(QString::number(controlUI->thruster7));
	controlUI->ui.thruster_val_8->setText(QString::number(controlUI->thruster8));

	controlUI->ui.p_val->setText(QString::number(controlUI->p));
	controlUI->ui.i_val->setText(QString::number(controlUI->i));
	controlUI->ui.d_val->setText(QString::number(controlUI->d));
	controlUI->ui.total_val->setText(QString::number(controlUI->total));
}

void updateGraph() {
	double x_val = (ros::Time::now() - controlUI->startTime).toSec();

	double x_org = controlUI->x_org;
	if (x_val - x_org > 20) {
		controlUI->ui.graph_canvas->graph(0)->removeDataBefore(x_org + 1);
		controlUI->ui.graph_canvas->graph(1)->removeDataBefore(x_org + 1);
		controlUI->x_org++;
	}

	controlUI->ui.graph_canvas->graph(0)->addData(x_val, controlUI->graphSetPt);//Set Point
	controlUI->ui.graph_canvas->graph(1)->addData(x_val, controlUI->graphOut);//Output
	controlUI->ui.graph_canvas->rescaleAxes();
	controlUI->ui.graph_canvas->replot();
}

string getdate() {
	int max_date = 12;
	time_t now;
	char date[max_date];
	date[0] = '\0';
	now = time(NULL);
	if (now != -1) { strftime(date, max_date, "%d%m%y", gmtime(&now)); }
	return date;
}

void saveFile() {
	QString selfilter = QString("*.bag");
	QString filename = QFileDialog::getSaveFileName(controlUI->window, QString("Open controls file"), QDir::currentPath(),
			QString(".txt files (*.txt);; All files (*.*)"), &selfilter);

	try {
		string strFilename = filename.toUtf8().constData();
		if (strFilename != "") {
			ROS_INFO("%s", strFilename.c_str());
			FILE* output;
			output = fopen(strFilename.c_str(), "w");
			for (int i = 0; i < numParams; i++) {
				string curParam = dynamicParams[i];
				fprintf(output, (curParam + ": %s\n").c_str(), controlUI->params[curParam].c_str());
			}
			fclose(output);
		} else {
			return;
		}
	} catch (int e) {
		QMessageBox::critical(controlUI->ui.centralwidget, "Error", "Something goes wrong!");
	}

	QMessageBox::information(controlUI->ui.centralwidget, "Information", "File successfully saved!");
}

void openTheFile() {
	QString selfilter = QString("*.bag");
	QString filename = QFileDialog::getOpenFileName(controlUI->window, QString("Open controls file"), QDir::currentPath(),
			QString(".txt files (*.txt);; All files (*.*)"), &selfilter);

	string strFilename = filename.toUtf8().constData();
	FILE* in;
	in = fopen(strFilename.c_str(), "r");
	if (in != NULL){
		string msg = "Loading file " + strFilename + "...";
		controlUI->ui.statusbar->showMessage(msg.c_str(), 3000);

		try {
			for (int i = 0; i < numParams; i++) {
				char val[256];
				fscanf(in, (dynamicParams[i] + ": %s").c_str(), &val);
				controlUI->params[dynamicParams[i]] = string(val);
			}
		} catch (int e) {
			QMessageBox::critical(controlUI->ui.centralwidget, "Error", "File's data is not correct");
		}
		controlUI->loadControlParams();
		fclose(in);
	} else {
		controlUI->ui.statusbar->showMessage("Cannot open file", 3000);
	}
}

//To enable all the check boxes
void enableButton(){
	//controlUI->enable = !controlUI->enable;
	if (controlUI->enable){
		controlUI->ui.fwd_check->setChecked(true);
		controlUI->ui.depth_check->setChecked(true);
		controlUI->ui.yaw_check->setChecked(true);
		controlUI->ui.sm_check->setChecked(true);
		controlUI->ui.roll_check->setChecked(true);
		controlUI->ui.pitch_check->setChecked(true);

		controlUI->updatePIDChecks(true, true, true, true, true, true);
	}
}

//Mouse clicked on graph so display data point
void mouseMoved(QMouseEvent *event) {
	ostringstream oss;
	oss << std::fixed << std::setprecision(3);

	double x = controlUI->ui.graph_canvas->xAxis->pixelToCoord(event->x());
	double y = controlUI->ui.graph_canvas->yAxis->pixelToCoord(event->y());

	oss << "Graph values: x: " << x << "\ty: " << y ;
	controlUI->ui.graphvalues->setText(QString::fromStdString(oss.str()));
}

//To send goal value
void fire() {
	if (!controlUI->live){
		QMessageBox::information(controlUI->ui.centralwidget, "Fire!", "Bang! Boom! Bam!");
	} else {
		ros::NodeHandle nh;
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::Config conf;

		bool forward, sidemove, heading, depth, pitch, roll;
		ros::param::get("/controller/forward_PID", forward);
		ros::param::get("/controller/sidemove_PID", sidemove);
		ros::param::get("/controller/heading_PID", heading);
		ros::param::get("/controller/depth_PID", depth);
		ros::param::get("/controller/pitch_PID", pitch);
		ros::param::get("/controller/roll_PID", roll);
		heading = heading || controlUI->ui.yaw_check->isChecked();
		forward = forward || controlUI->ui.fwd_check->isChecked();
		depth = depth || controlUI->ui.depth_check->isChecked();
		sidemove = sidemove || controlUI->ui.sm_check->isChecked();
		pitch = pitch || controlUI->ui.pitch_check->isChecked();
		roll = roll || controlUI->ui.roll_check->isChecked();

		double value = controlUI->ui.goal_val->text().toDouble();
		if (controlUI->graphType == "Depth") {
			depth = true;
			controlUI->updateParameter("depth_setpoint", value, conf);
		} else if (controlUI->graphType == "Pitch") {
			pitch = true;
			controlUI->updateParameter("pitch_setpoint", value, conf);
		} else if (controlUI->graphType == "Heading") {
			heading = true;
			controlUI->updateParameter("heading_setpoint", value, conf);
		} else if (controlUI->graphType == "Roll") {
			roll = true;
			controlUI->updateParameter("roll_setpoint", value, conf);
		} else if (controlUI->graphType == "Side") {
			sidemove = true;
			controlUI->updateParameter("sidemove_setpoint", value, conf);
		} else if (controlUI->graphType == "Forward") {
			forward = true;
			controlUI->updateParameter("forward_setpoint", value, conf);
		}

		//controlClient.call(srv);
		controlUI->updatePIDChecks(depth, heading, forward, sidemove, roll, pitch);
		srv_req.config = conf;
		ros::service::call("/controller/set_parameters", srv_req, srv_resp);

		controlUI->ui.statusbar->showMessage("Goal setpoint sent!", 3000);
	}
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
		const srmauv_msgs::ControllerResultConstPtr& result) {
	const char* status = state.toString().c_str();
	ROS_INFO("Finished in state [%s]", status);
	controlUI->ui.statusbar->showMessage(status, 3000);
}

// Called once when the goal becomes active
void activeCb() {
	ROS_INFO("Goal just went active");
	controlUI->ui.statusbar->showMessage("Goal just went active", 3000);
}

// Called every time feedback is received for the goal
void feedbackCb(const srmauv_msgs::ControllerFeedbackConstPtr& feedback) {
	ROS_INFO("Got Feedback of length");
}

// Send the advanced parameters
void sendButton(){
	if (!controlUI->live){
		QMessageBox::information(controlUI->ui.centralwidget, "Send!", "Bang! Boom! Bam!");
	}
	else {
		ros::NodeHandle nh;
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::Config conf;

		bool forward, sidemove, heading, depth, pitch, roll;
		ros::param::get("/controller/forward_PID", forward);
		ros::param::get("/controller/sidemove_PID", sidemove);
		ros::param::get("/controller/heading_PID", heading);
		ros::param::get("/controller/depth_PID", depth);
		ros::param::get("/controller/pitch_PID", pitch);
		ros::param::get("/controller/roll_PID", roll);
		heading = heading || controlUI->ui.yaw_check->isChecked();
		forward = forward || controlUI->ui.fwd_check->isChecked();
		depth = depth || controlUI->ui.depth_check->isChecked();
		sidemove = sidemove || controlUI->ui.sm_check->isChecked();
		pitch = pitch || controlUI->ui.pitch_check->isChecked();
		roll = roll || controlUI->ui.roll_check->isChecked();

		controlUI->updatePIDChecks(depth, heading, forward, sidemove, roll, pitch);

		if (depth) {
			controlUI->updateParameter("depth_setpoint", controlUI->ui.depth_val->text().toDouble(), conf);
		}
		if (pitch) {
			controlUI->updateParameter("pitch_setpoint", controlUI->ui.pitch_val->text().toDouble(), conf);
		}
		if (heading) {
			controlUI->updateParameter("heading_setpoint", controlUI->ui.yaw_val->text().toDouble(), conf);
		}
		if (roll) {
			controlUI->updateParameter("roll_setpoint", controlUI->ui.roll_val->text().toDouble(), conf);
		}
		if (sidemove) {
			controlUI->updateParameter("sidemove_setpoint", controlUI->ui.sm_val->text().toDouble(), conf);
		}
		if (forward) {
			controlUI->updateParameter("forward_setpoint", controlUI->ui.fwd_val->text().toDouble(), conf);
		}

		srv_req.config = conf;
		controlUI->ui.statusbar->showMessage("Status: Waiting for dynamic reconfigure service", 3000);
		ros::service::waitForService("/controller/set_parameters", ros::Duration(0.5));
		ros::service::call("/controller/set_parameters", srv_req, srv_resp);
		controlUI->ui.statusbar->showMessage("Advanced goals sent!", 3000);
	}
}

// Tune the control parameters by publishing them
void tuneButton(){
	if (!controlUI->live){
		QMessageBox::information(controlUI->ui.centralwidget, "Tune!", "Twinkle Twinkle Little Star~");
	} else {
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::Config conf;

		for (int i = controlUI->startIndex; i < controlUI->endIndex; i++) {
			int offsetIndex = i % 5;
			if (paramsTypes[offsetIndex] == "int_t") {
				int temp = controlUI->configureWidgets[offsetIndex]->text().toInt();
				controlUI->updateParameter(dynamicParams[i], temp, conf);
				controlUI->params[dynamicParams[i]] = boost::lexical_cast<string, int>(temp);
			} else if (paramsTypes[offsetIndex] == "double_t") {
				double temp = controlUI->configureWidgets[offsetIndex]->text().toDouble();
				controlUI->updateParameter(dynamicParams[i], temp, conf);
				controlUI->params[dynamicParams[i]] = boost::lexical_cast<string, double>(temp);
			}
		}

		srv_req.config = conf;
		controlUI->ui.statusbar->showMessage("Status: Waiting for dynamic reconfigure service", 3000);
		ros::service::waitForService("/controller/set_parameters", ros::Duration(0.5));
		ros::service::call("/controller/set_parameters", srv_req, srv_resp);
		controlUI->autoSave();
		controlUI->loadDynamicParams();
		controlUI->loadControlParams();
		controlUI->ui.statusbar->showMessage("Status: Finished Tuning", 3000);
	}
}

void resetGoalsUI() {
	controlUI->ui.fwd_check->setDisabled(false);
	controlUI->ui.yaw_check->setDisabled(false);
	controlUI->ui.depth_check->setDisabled(false);
	controlUI->ui.sm_check->setDisabled(false);
	controlUI->ui.roll_check->setDisabled(false);
	controlUI->ui.pitch_check->setDisabled(false);

	controlUI->ui.fwd_check->setChecked(false);
	controlUI->ui.yaw_check->setChecked(false);
	controlUI->ui.depth_check->setChecked(false);
	controlUI->ui.sm_check->setChecked(false);
	controlUI->ui.roll_check->setChecked(false);
	controlUI->ui.pitch_check->setChecked(false);

	controlUI->ui.fwd_val->setDisabled(false);
	controlUI->ui.yaw_val->setDisabled(false);
	controlUI->ui.depth_val->setDisabled(false);
	controlUI->ui.sm_val->setDisabled(false);
	controlUI->ui.roll_val->setDisabled(false);
	controlUI->ui.pitch_val->setDisabled(false);
}

void graphTypeChanged(const QString& type) {
	controlUI->ui.graph_canvas->graph(0)->clearData();
	controlUI->ui.graph_canvas->graph(1)->clearData();
	controlUI->x_org = 0;
	controlUI->startTime = ros::Time::now();
	controlUI->graphType = type.toStdString();

	resetGoalsUI();
	controlUI->loadPIDChecks();
	if (controlUI->graphType == "Depth") {
		controlUI->startIndex = depthIndex;
		controlUI->ui.depth_check->setDisabled(true);
		controlUI->ui.depth_val->setDisabled(true);
	} else if (controlUI->graphType == "Heading") {
		controlUI->startIndex = headingIndex;
		controlUI->ui.yaw_check->setDisabled(true);
		controlUI->ui.yaw_val->setDisabled(true);
	} else if (controlUI->graphType == "Forward") {
		controlUI->startIndex = forwardIndex;
		controlUI->ui.fwd_check->setDisabled(true);
		controlUI->ui.fwd_val->setDisabled(true);
	} else if (controlUI->graphType == "Side") {
		controlUI->startIndex = sidemoveIndex;
		controlUI->ui.sm_check->setDisabled(true);
		controlUI->ui.sm_val->setDisabled(true);
	} else if (controlUI->graphType == "Roll") {
		controlUI->startIndex = rollIndex;
		controlUI->ui.roll_check->setDisabled(true);
		controlUI->ui.roll_val->setDisabled(true);
	} else if (controlUI->graphType == "Pitch") {
		controlUI->startIndex = pitchIndex;
		controlUI->ui.pitch_check->setDisabled(true);
		controlUI->ui.pitch_val->setDisabled(true);
	}
	controlUI->endIndex = controlUI->startIndex + 5; //Exclusive

	controlUI->loadControlParams();
}

void disableButton() {
	controlUI->ui.fwd_check->setChecked(false);
	controlUI->ui.depth_check->setChecked(false);
	controlUI->ui.yaw_check->setChecked(false);
	controlUI->ui.sm_check->setChecked(false);
	controlUI->ui.roll_check->setChecked(false);
	controlUI->ui.pitch_check->setChecked(false);

	controlUI->updatePIDChecks(false, false, false, false, false, false);
}

void refreshButton() {
	controlUI->graph_update.shutdown();
	controlUI->thruster_sub.shutdown();
	controlUI->errorSub.shutdown();
	controlUI->pidInfoSub.shutdown();
	controlUI->subscribeToData();

	controlUI->loadPIDChecks();
	controlUI->loadDynamicParams();
	controlUI->loadControlParams();
}

void updateIncomingParams() {
	for (int i = 0; i < numParams; i++) {
		string strVal = "";
		if (paramsTypes[i%5] == "double_t") {
			double val = 0.0;
			ros::param::getCached("/controller/" + dynamicParams[i], val);
			strVal = boost::lexical_cast<string>(val);
		} else if (paramsTypes[i%5] == "int_t") {
			int val = 0;
			ros::param::getCached("/controller/" + dynamicParams[i], val);
			strVal = boost::lexical_cast<string>(val);
		}

		if (controlUI->params[dynamicParams[i]] != strVal) {
			controlUI->params[dynamicParams[i]] = strVal;
			if (i >= controlUI->startIndex && i < controlUI->endIndex) {
				controlUI->configureWidgets[i % 5]->setText(QString::fromStdString(strVal));
			}
		}
	}
}

void quit(int sig)
{
	std::cout<<"Quiting"<<std::endl;
	ros::shutdown();
	QApplication::quit();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tuner", ros::init_options::AnonymousName);

	//Initiate QAppication and UI
	QApplication app(argc, argv);

	ControlUI lControlUI;
	controlUI = &lControlUI;



	QObject::connect(controlUI->ui.actionSave, &QAction::triggered, saveFile);
	QObject::connect(controlUI->ui.fireButton, &QAbstractButton::released, fire);
	QObject::connect(controlUI->ui.actionOpen, &QAction::triggered, openTheFile);
	QObject::connect(controlUI->ui.enabledButton, &QAbstractButton::released, enableButton);
	QObject::connect(controlUI->ui.tuneButton, &QAbstractButton::released, tuneButton);
	QObject::connect(controlUI->ui.sendButton, &QAbstractButton::released, sendButton);
	QObject::connect(controlUI->ui.graphType,
			static_cast<void (QComboBox::*)(const QString&)>(&QComboBox::currentIndexChanged),
			graphTypeChanged);
	QObject::connect(controlUI->ui.disableButton, &QAbstractButton::released, disableButton);
	QObject::connect(controlUI->ui.refreshButton, &QAbstractButton::released, refreshButton);

	controlUI->ui.depth_check->setDisabled(true);
	controlUI->ui.depth_val->setDisabled(true);

	QTimer *timer = new QTimer();
	QObject::connect(timer, &QTimer::timeout, updateGraph);
	timer->start(50);

	QTimer *statusTimer = new QTimer();
	QObject::connect(statusTimer, &QTimer::timeout, updateStatus);
	statusTimer->start(50);

	QTimer *paramsTimer = new QTimer();
	QObject::connect(paramsTimer, &QTimer::timeout, updateIncomingParams);
	paramsTimer->start(1000);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	controlUI->window->show();

	signal(SIGINT, quit);
	int status = app.exec();
	delete timer;
	delete statusTimer;
	return status;
}

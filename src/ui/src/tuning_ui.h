/********************************************************************************
** Form generated from reading UI file 'tuningui.ui'
**
** Created by: Qt User Interface Compiler version 5.1.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef TUNING_UI_H
#define TUNING_UI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_ControlSysUI
{
public:
    QAction *actionSave;
    QAction *actionOpen;
    QAction *actionQuit;
    QWidget *centralwidget;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *graph;
    QCustomPlot *graph_canvas;
    QLabel *graph_label;
    QFrame *telemetry_box;
    QLabel *setpt_label;
    QLabel *sensor_label;
    QLabel *error_label;
    QLabel *KP_label;
    QLabel *KI_label;
    QLabel *KD_label;
    QLabel *output_label;
    QLabel *thruster_label;
    QLabel *setpt_val;
    QLabel *sensor_val;
    QLabel *error_val;
    QLabel *p_val;
    QLabel *i_val;
    QLabel *d_val;
    QLabel *total_val;
    QLabel *thruster_val_1;
    QLabel *telemetry_label;
    QLabel *thruster_val_2;
    QLabel *thruster_val_3;
    QLabel *thruster_val_4;
    QLabel *thruster_val_8;
    QLabel *thruster_val_6;
    QLabel *thruster_val_7;
    QLabel *thruster_val_5;
    QLabel *thruster_label_2;
    QLabel *thruster_label_3;
    QLabel *thruster_label_4;
    QLabel *thruster_label_5;
    QLabel *thruster_label_6;
    QLabel *thruster_label_7;
    QLabel *thruster_label_8;
    QLabel *thruster_label_9;
    QFrame *controparam_frame;
    QLabel *actmin_label;
    QLabel *actmax_label;
    QLabel *con_KD_label;
    QLabel *con_KI_label;
    QLabel *Controlpara_label;
    QLabel *con_KP_label;
    QLineEdit *conKpVal;
    QLineEdit *conTiVal;
    QLineEdit *conMinVal;
    QLineEdit *conMaxVal;
    QPushButton *tuneButton;
    QLineEdit *conTdVal;
    QFrame *advanced_frame;
    QLabel *Fwd_label;
    QCheckBox *fwd_check;
    QLineEdit *fwd_val;
    QLabel *Depth_label;
    QCheckBox *depth_check;
    QLineEdit *depth_val;
    QLineEdit *yaw_val;
    QLabel *yaw_label;
    QCheckBox *yaw_check;
    QLineEdit *sm_val;
    QCheckBox *sm_check;
    QLabel *sm_label;
    QPushButton *sendButton;
    QPushButton *enabledButton;
    QLabel *sm_label_2;
    QCheckBox *roll_check;
    QLineEdit *roll_val;
    QLineEdit *pitch_val;
    QLabel *sm_label_3;
    QCheckBox *pitch_check;
    QLabel *graphvalues;
    QFrame *frame;
    QPushButton *fireButton;
    QLineEdit *goal_val;
    QLabel *Goal_label;
    QLabel *DOF_label;
    QComboBox *graphType;
    QFrame *line;
    QPushButton *disableButton;
    QPushButton *refreshButton;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *ControlSysUI)
    {
        if (ControlSysUI->objectName().isEmpty())
            ControlSysUI->setObjectName(QStringLiteral("ControlSysUI"));
        ControlSysUI->resize(945, 592);
        ControlSysUI->setTabShape(QTabWidget::Rounded);
        actionSave = new QAction(ControlSysUI);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionOpen = new QAction(ControlSysUI);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionQuit = new QAction(ControlSysUI);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        centralwidget = new QWidget(ControlSysUI);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayoutWidget = new QWidget(centralwidget);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(20, 50, 371, 431));
        graph = new QHBoxLayout(horizontalLayoutWidget);
        graph->setObjectName(QStringLiteral("graph"));
        graph->setContentsMargins(0, 0, 0, 0);
        graph_canvas = new QCustomPlot(horizontalLayoutWidget);
        graph_canvas->setObjectName(QStringLiteral("graph_canvas"));
        graph_canvas->setCursor(QCursor(Qt::CrossCursor));

        graph->addWidget(graph_canvas);

        graph_label = new QLabel(centralwidget);
        graph_label->setObjectName(QStringLiteral("graph_label"));
        graph_label->setGeometry(QRect(20, 20, 66, 17));
        telemetry_box = new QFrame(centralwidget);
        telemetry_box->setObjectName(QStringLiteral("telemetry_box"));
        telemetry_box->setGeometry(QRect(400, 10, 531, 151));
        telemetry_box->setFrameShape(QFrame::StyledPanel);
        setpt_label = new QLabel(telemetry_box);
        setpt_label->setObjectName(QStringLiteral("setpt_label"));
        setpt_label->setGeometry(QRect(20, 40, 51, 17));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        setpt_label->setFont(font);
        sensor_label = new QLabel(telemetry_box);
        sensor_label->setObjectName(QStringLiteral("sensor_label"));
        sensor_label->setGeometry(QRect(20, 70, 51, 17));
        sensor_label->setFont(font);
        error_label = new QLabel(telemetry_box);
        error_label->setObjectName(QStringLiteral("error_label"));
        error_label->setGeometry(QRect(20, 100, 51, 17));
        error_label->setFont(font);
        KP_label = new QLabel(telemetry_box);
        KP_label->setObjectName(QStringLiteral("KP_label"));
        KP_label->setGeometry(QRect(170, 40, 21, 17));
        KP_label->setFont(font);
        KI_label = new QLabel(telemetry_box);
        KI_label->setObjectName(QStringLiteral("KI_label"));
        KI_label->setGeometry(QRect(170, 70, 21, 17));
        KI_label->setFont(font);
        KD_label = new QLabel(telemetry_box);
        KD_label->setObjectName(QStringLiteral("KD_label"));
        KD_label->setGeometry(QRect(170, 100, 21, 17));
        KD_label->setFont(font);
        output_label = new QLabel(telemetry_box);
        output_label->setObjectName(QStringLiteral("output_label"));
        output_label->setGeometry(QRect(150, 130, 51, 17));
        output_label->setFont(font);
        thruster_label = new QLabel(telemetry_box);
        thruster_label->setObjectName(QStringLiteral("thruster_label"));
        thruster_label->setGeometry(QRect(270, 40, 61, 17));
        thruster_label->setFont(font);
        setpt_val = new QLabel(telemetry_box);
        setpt_val->setObjectName(QStringLiteral("setpt_val"));
        setpt_val->setGeometry(QRect(80, 40, 66, 17));
        sensor_val = new QLabel(telemetry_box);
        sensor_val->setObjectName(QStringLiteral("sensor_val"));
        sensor_val->setGeometry(QRect(80, 70, 66, 17));
        error_val = new QLabel(telemetry_box);
        error_val->setObjectName(QStringLiteral("error_val"));
        error_val->setGeometry(QRect(80, 100, 66, 17));
        p_val = new QLabel(telemetry_box);
        p_val->setObjectName(QStringLiteral("p_val"));
        p_val->setGeometry(QRect(190, 40, 66, 17));
        i_val = new QLabel(telemetry_box);
        i_val->setObjectName(QStringLiteral("i_val"));
        i_val->setGeometry(QRect(190, 70, 66, 17));
        d_val = new QLabel(telemetry_box);
        d_val->setObjectName(QStringLiteral("d_val"));
        d_val->setGeometry(QRect(190, 100, 66, 17));
        total_val = new QLabel(telemetry_box);
        total_val->setObjectName(QStringLiteral("total_val"));
        total_val->setGeometry(QRect(190, 130, 66, 17));
        thruster_val_1 = new QLabel(telemetry_box);
        thruster_val_1->setObjectName(QStringLiteral("thruster_val_1"));
        thruster_val_1->setGeometry(QRect(360, 40, 66, 17));
        telemetry_label = new QLabel(telemetry_box);
        telemetry_label->setObjectName(QStringLiteral("telemetry_label"));
        telemetry_label->setGeometry(QRect(20, 10, 81, 17));
        QFont font1;
        font1.setPointSize(12);
        font1.setBold(true);
        font1.setItalic(true);
        font1.setUnderline(true);
        font1.setWeight(75);
        telemetry_label->setFont(font1);
        thruster_val_2 = new QLabel(telemetry_box);
        thruster_val_2->setObjectName(QStringLiteral("thruster_val_2"));
        thruster_val_2->setGeometry(QRect(360, 70, 66, 17));
        thruster_val_3 = new QLabel(telemetry_box);
        thruster_val_3->setObjectName(QStringLiteral("thruster_val_3"));
        thruster_val_3->setGeometry(QRect(360, 100, 66, 17));
        thruster_val_4 = new QLabel(telemetry_box);
        thruster_val_4->setObjectName(QStringLiteral("thruster_val_4"));
        thruster_val_4->setGeometry(QRect(360, 130, 66, 17));
        thruster_val_8 = new QLabel(telemetry_box);
        thruster_val_8->setObjectName(QStringLiteral("thruster_val_8"));
        thruster_val_8->setGeometry(QRect(450, 130, 66, 17));
        thruster_val_6 = new QLabel(telemetry_box);
        thruster_val_6->setObjectName(QStringLiteral("thruster_val_6"));
        thruster_val_6->setGeometry(QRect(450, 70, 66, 17));
        thruster_val_7 = new QLabel(telemetry_box);
        thruster_val_7->setObjectName(QStringLiteral("thruster_val_7"));
        thruster_val_7->setGeometry(QRect(450, 100, 71, 21));
        thruster_val_5 = new QLabel(telemetry_box);
        thruster_val_5->setObjectName(QStringLiteral("thruster_val_5"));
        thruster_val_5->setGeometry(QRect(450, 40, 66, 17));
        thruster_label_2 = new QLabel(telemetry_box);
        thruster_label_2->setObjectName(QStringLiteral("thruster_label_2"));
        thruster_label_2->setGeometry(QRect(340, 40, 21, 17));
        thruster_label_2->setFont(font);
        thruster_label_3 = new QLabel(telemetry_box);
        thruster_label_3->setObjectName(QStringLiteral("thruster_label_3"));
        thruster_label_3->setGeometry(QRect(340, 70, 21, 17));
        thruster_label_3->setFont(font);
        thruster_label_4 = new QLabel(telemetry_box);
        thruster_label_4->setObjectName(QStringLiteral("thruster_label_4"));
        thruster_label_4->setGeometry(QRect(340, 100, 21, 17));
        thruster_label_4->setFont(font);
        thruster_label_5 = new QLabel(telemetry_box);
        thruster_label_5->setObjectName(QStringLiteral("thruster_label_5"));
        thruster_label_5->setGeometry(QRect(340, 130, 21, 17));
        thruster_label_5->setFont(font);
        thruster_label_6 = new QLabel(telemetry_box);
        thruster_label_6->setObjectName(QStringLiteral("thruster_label_6"));
        thruster_label_6->setGeometry(QRect(430, 40, 21, 17));
        thruster_label_6->setFont(font);
        thruster_label_7 = new QLabel(telemetry_box);
        thruster_label_7->setObjectName(QStringLiteral("thruster_label_7"));
        thruster_label_7->setGeometry(QRect(430, 70, 21, 17));
        thruster_label_7->setFont(font);
        thruster_label_8 = new QLabel(telemetry_box);
        thruster_label_8->setObjectName(QStringLiteral("thruster_label_8"));
        thruster_label_8->setGeometry(QRect(430, 100, 21, 17));
        thruster_label_8->setFont(font);
        thruster_label_9 = new QLabel(telemetry_box);
        thruster_label_9->setObjectName(QStringLiteral("thruster_label_9"));
        thruster_label_9->setGeometry(QRect(430, 130, 21, 17));
        thruster_label_9->setFont(font);
        controparam_frame = new QFrame(centralwidget);
        controparam_frame->setObjectName(QStringLiteral("controparam_frame"));
        controparam_frame->setGeometry(QRect(400, 410, 531, 131));
        controparam_frame->setFrameShape(QFrame::StyledPanel);
        controparam_frame->setFrameShadow(QFrame::Raised);
        actmin_label = new QLabel(controparam_frame);
        actmin_label->setObjectName(QStringLiteral("actmin_label"));
        actmin_label->setGeometry(QRect(170, 40, 61, 17));
        actmin_label->setFont(font);
        actmax_label = new QLabel(controparam_frame);
        actmax_label->setObjectName(QStringLiteral("actmax_label"));
        actmax_label->setGeometry(QRect(170, 80, 71, 17));
        actmax_label->setFont(font);
        con_KD_label = new QLabel(controparam_frame);
        con_KD_label->setObjectName(QStringLiteral("con_KD_label"));
        con_KD_label->setGeometry(QRect(20, 100, 51, 17));
        con_KD_label->setFont(font);
        con_KI_label = new QLabel(controparam_frame);
        con_KI_label->setObjectName(QStringLiteral("con_KI_label"));
        con_KI_label->setGeometry(QRect(20, 70, 51, 17));
        con_KI_label->setFont(font);
        Controlpara_label = new QLabel(controparam_frame);
        Controlpara_label->setObjectName(QStringLiteral("Controlpara_label"));
        Controlpara_label->setGeometry(QRect(20, 10, 151, 17));
        con_KP_label = new QLabel(controparam_frame);
        con_KP_label->setObjectName(QStringLiteral("con_KP_label"));
        con_KP_label->setGeometry(QRect(20, 40, 51, 17));
        con_KP_label->setFont(font);
        conKpVal = new QLineEdit(controparam_frame);
        conKpVal->setObjectName(QStringLiteral("conKpVal"));
        conKpVal->setGeometry(QRect(60, 40, 81, 27));
        conTiVal = new QLineEdit(controparam_frame);
        conTiVal->setObjectName(QStringLiteral("conTiVal"));
        conTiVal->setGeometry(QRect(60, 70, 81, 27));
        conMinVal = new QLineEdit(controparam_frame);
        conMinVal->setObjectName(QStringLiteral("conMinVal"));
        conMinVal->setGeometry(QRect(250, 40, 81, 27));
        conMaxVal = new QLineEdit(controparam_frame);
        conMaxVal->setObjectName(QStringLiteral("conMaxVal"));
        conMaxVal->setGeometry(QRect(250, 80, 81, 27));
        tuneButton = new QPushButton(controparam_frame);
        tuneButton->setObjectName(QStringLiteral("tuneButton"));
        tuneButton->setGeometry(QRect(390, 80, 98, 31));
        conTdVal = new QLineEdit(controparam_frame);
        conTdVal->setObjectName(QStringLiteral("conTdVal"));
        conTdVal->setGeometry(QRect(60, 100, 81, 27));
        advanced_frame = new QFrame(centralwidget);
        advanced_frame->setObjectName(QStringLiteral("advanced_frame"));
        advanced_frame->setGeometry(QRect(630, 170, 301, 231));
        advanced_frame->setFrameShape(QFrame::StyledPanel);
        advanced_frame->setFrameShadow(QFrame::Raised);
        Fwd_label = new QLabel(advanced_frame);
        Fwd_label->setObjectName(QStringLiteral("Fwd_label"));
        Fwd_label->setGeometry(QRect(60, 10, 51, 17));
        fwd_check = new QCheckBox(advanced_frame);
        fwd_check->setObjectName(QStringLiteral("fwd_check"));
        fwd_check->setGeometry(QRect(120, 10, 41, 22));
        fwd_val = new QLineEdit(advanced_frame);
        fwd_val->setObjectName(QStringLiteral("fwd_val"));
        fwd_val->setGeometry(QRect(170, 10, 101, 27));
        Depth_label = new QLabel(advanced_frame);
        Depth_label->setObjectName(QStringLiteral("Depth_label"));
        Depth_label->setGeometry(QRect(60, 40, 51, 17));
        depth_check = new QCheckBox(advanced_frame);
        depth_check->setObjectName(QStringLiteral("depth_check"));
        depth_check->setGeometry(QRect(120, 40, 41, 22));
        depth_check->setChecked(false);
        depth_val = new QLineEdit(advanced_frame);
        depth_val->setObjectName(QStringLiteral("depth_val"));
        depth_val->setGeometry(QRect(170, 40, 101, 27));
        yaw_val = new QLineEdit(advanced_frame);
        yaw_val->setObjectName(QStringLiteral("yaw_val"));
        yaw_val->setGeometry(QRect(170, 70, 101, 27));
        yaw_label = new QLabel(advanced_frame);
        yaw_label->setObjectName(QStringLiteral("yaw_label"));
        yaw_label->setGeometry(QRect(60, 70, 61, 17));
        yaw_check = new QCheckBox(advanced_frame);
        yaw_check->setObjectName(QStringLiteral("yaw_check"));
        yaw_check->setGeometry(QRect(120, 70, 41, 22));
        sm_val = new QLineEdit(advanced_frame);
        sm_val->setObjectName(QStringLiteral("sm_val"));
        sm_val->setGeometry(QRect(170, 100, 101, 27));
        sm_check = new QCheckBox(advanced_frame);
        sm_check->setObjectName(QStringLiteral("sm_check"));
        sm_check->setGeometry(QRect(120, 100, 41, 22));
        sm_label = new QLabel(advanced_frame);
        sm_label->setObjectName(QStringLiteral("sm_label"));
        sm_label->setGeometry(QRect(60, 100, 51, 17));
        sendButton = new QPushButton(advanced_frame);
        sendButton->setObjectName(QStringLiteral("sendButton"));
        sendButton->setGeometry(QRect(170, 190, 101, 31));
        enabledButton = new QPushButton(advanced_frame);
        enabledButton->setObjectName(QStringLiteral("enabledButton"));
        enabledButton->setGeometry(QRect(60, 190, 61, 31));
        QFont font2;
        font2.setPointSize(10);
        font2.setBold(true);
        font2.setItalic(true);
        font2.setWeight(75);
        enabledButton->setFont(font2);
        sm_label_2 = new QLabel(advanced_frame);
        sm_label_2->setObjectName(QStringLiteral("sm_label_2"));
        sm_label_2->setEnabled(true);
        sm_label_2->setGeometry(QRect(60, 130, 51, 17));
        roll_check = new QCheckBox(advanced_frame);
        roll_check->setObjectName(QStringLiteral("roll_check"));
        roll_check->setEnabled(true);
        roll_check->setGeometry(QRect(120, 130, 41, 22));
        roll_val = new QLineEdit(advanced_frame);
        roll_val->setObjectName(QStringLiteral("roll_val"));
        roll_val->setEnabled(true);
        roll_val->setGeometry(QRect(170, 130, 101, 27));
        pitch_val = new QLineEdit(advanced_frame);
        pitch_val->setObjectName(QStringLiteral("pitch_val"));
        pitch_val->setEnabled(true);
        pitch_val->setGeometry(QRect(170, 160, 101, 27));
        sm_label_3 = new QLabel(advanced_frame);
        sm_label_3->setObjectName(QStringLiteral("sm_label_3"));
        sm_label_3->setEnabled(true);
        sm_label_3->setGeometry(QRect(60, 160, 51, 17));
        pitch_check = new QCheckBox(advanced_frame);
        pitch_check->setObjectName(QStringLiteral("pitch_check"));
        pitch_check->setEnabled(true);
        pitch_check->setGeometry(QRect(120, 160, 41, 22));
        graphvalues = new QLabel(centralwidget);
        graphvalues->setObjectName(QStringLiteral("graphvalues"));
        graphvalues->setGeometry(QRect(20, 490, 321, 21));
        QFont font3;
        font3.setPointSize(11);
        font3.setBold(true);
        font3.setWeight(75);
        graphvalues->setFont(font3);
        graphvalues->setTextFormat(Qt::AutoText);
        frame = new QFrame(centralwidget);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(400, 170, 221, 231));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        fireButton = new QPushButton(frame);
        fireButton->setObjectName(QStringLiteral("fireButton"));
        fireButton->setGeometry(QRect(80, 130, 101, 27));
        fireButton->setAutoFillBackground(true);
        fireButton->setCheckable(false);
        goal_val = new QLineEdit(frame);
        goal_val->setObjectName(QStringLiteral("goal_val"));
        goal_val->setGeometry(QRect(80, 100, 101, 27));
        goal_val->setCursor(QCursor(Qt::IBeamCursor));
        Goal_label = new QLabel(frame);
        Goal_label->setObjectName(QStringLiteral("Goal_label"));
        Goal_label->setGeometry(QRect(30, 100, 51, 17));
        DOF_label = new QLabel(frame);
        DOF_label->setObjectName(QStringLiteral("DOF_label"));
        DOF_label->setGeometry(QRect(30, 30, 51, 17));
        graphType = new QComboBox(frame);
        graphType->setObjectName(QStringLiteral("graphType"));
        graphType->setGeometry(QRect(80, 20, 101, 31));
        line = new QFrame(frame);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(20, 70, 181, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        disableButton = new QPushButton(frame);
        disableButton->setObjectName(QStringLiteral("disableButton"));
        disableButton->setGeometry(QRect(150, 190, 61, 31));
        disableButton->setFont(font2);
        refreshButton = new QPushButton(centralwidget);
        refreshButton->setObjectName(QStringLiteral("refreshButton"));
        refreshButton->setGeometry(QRect(290, 10, 99, 27));
        ControlSysUI->setCentralWidget(centralwidget);
        frame->raise();
        horizontalLayoutWidget->raise();
        graph_label->raise();
        telemetry_box->raise();
        controparam_frame->raise();
        advanced_frame->raise();
        graphvalues->raise();
        refreshButton->raise();
        menubar = new QMenuBar(ControlSysUI);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 945, 20));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        ControlSysUI->setMenuBar(menubar);
        statusbar = new QStatusBar(ControlSysUI);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        ControlSysUI->setStatusBar(statusbar);
        QWidget::setTabOrder(goal_val, fireButton);
        QWidget::setTabOrder(fireButton, fwd_check);
        QWidget::setTabOrder(fwd_check, fwd_val);
        QWidget::setTabOrder(fwd_val, depth_check);
        QWidget::setTabOrder(depth_check, depth_val);
        QWidget::setTabOrder(depth_val, yaw_check);
        QWidget::setTabOrder(yaw_check, yaw_val);
        QWidget::setTabOrder(yaw_val, sm_check);
        QWidget::setTabOrder(sm_check, sm_val);
        QWidget::setTabOrder(sm_val, roll_check);
        QWidget::setTabOrder(roll_check, roll_val);
        QWidget::setTabOrder(roll_val, pitch_check);
        QWidget::setTabOrder(pitch_check, pitch_val);
        QWidget::setTabOrder(pitch_val, sendButton);
        QWidget::setTabOrder(sendButton, conKpVal);
        QWidget::setTabOrder(conKpVal, conTiVal);
        QWidget::setTabOrder(conTiVal, conTdVal);
        QWidget::setTabOrder(conTdVal, conMinVal);
        QWidget::setTabOrder(conMinVal, conMaxVal);
        QWidget::setTabOrder(conMaxVal, tuneButton);
        QWidget::setTabOrder(tuneButton, enabledButton);
        QWidget::setTabOrder(enabledButton, graphType);
        QWidget::setTabOrder(graphType, disableButton);
        QWidget::setTabOrder(disableButton, refreshButton);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(actionSave);
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionQuit);

        retranslateUi(ControlSysUI);
        QObject::connect(actionQuit, SIGNAL(triggered()), ControlSysUI, SLOT(close()));

        QMetaObject::connectSlotsByName(ControlSysUI);
    } // setupUi

    void retranslateUi(QMainWindow *ControlSysUI)
    {
        ControlSysUI->setWindowTitle(QApplication::translate("ControlSysUI", "Control System UI", 0));
        actionSave->setText(QApplication::translate("ControlSysUI", "&Save", 0));
        actionSave->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+S", 0));
        actionOpen->setText(QApplication::translate("ControlSysUI", "&Open", 0));
        actionOpen->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+O", 0));
        actionQuit->setText(QApplication::translate("ControlSysUI", "&Quit", 0));
        actionQuit->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+Q", 0));
        graph_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-weight:600;\">Graph</span></p></body></html>", 0));
        setpt_label->setText(QApplication::translate("ControlSysUI", "Set pt", 0));
        sensor_label->setText(QApplication::translate("ControlSysUI", "Sensor", 0));
        error_label->setText(QApplication::translate("ControlSysUI", "Error", 0));
        KP_label->setText(QApplication::translate("ControlSysUI", "p", 0));
        KI_label->setText(QApplication::translate("ControlSysUI", "i", 0));
        KD_label->setText(QApplication::translate("ControlSysUI", "d", 0));
        output_label->setText(QApplication::translate("ControlSysUI", "Total", 0));
        thruster_label->setText(QApplication::translate("ControlSysUI", "Thruster", 0));
        setpt_val->setText(QString());
        sensor_val->setText(QString());
        error_val->setText(QString());
        p_val->setText(QString());
        i_val->setText(QString());
        d_val->setText(QString());
        total_val->setText(QString());
        thruster_val_1->setText(QApplication::translate("ControlSysUI", "0", 0));
        telemetry_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-style:normal;\">Telemetry</span></p></body></html>", 0));
        thruster_val_2->setText(QApplication::translate("ControlSysUI", "0", 0));
        thruster_val_3->setText(QApplication::translate("ControlSysUI", "0", 0));
        thruster_val_4->setText(QApplication::translate("ControlSysUI", "0", 0));
        thruster_val_8->setText(QApplication::translate("ControlSysUI", "0", 0));
        thruster_val_6->setText(QApplication::translate("ControlSysUI", "0", 0));
        thruster_val_7->setText(QApplication::translate("ControlSysUI", "0", 0));
        thruster_val_5->setText(QApplication::translate("ControlSysUI", "0", 0));
        thruster_label_2->setText(QApplication::translate("ControlSysUI", "1:", 0));
        thruster_label_3->setText(QApplication::translate("ControlSysUI", "2:", 0));
        thruster_label_4->setText(QApplication::translate("ControlSysUI", "3:", 0));
        thruster_label_5->setText(QApplication::translate("ControlSysUI", "4:", 0));
        thruster_label_6->setText(QApplication::translate("ControlSysUI", "5:", 0));
        thruster_label_7->setText(QApplication::translate("ControlSysUI", "6:", 0));
        thruster_label_8->setText(QApplication::translate("ControlSysUI", "7:", 0));
        thruster_label_9->setText(QApplication::translate("ControlSysUI", "8:", 0));
        actmin_label->setText(QApplication::translate("ControlSysUI", "Act_Min", 0));
        actmax_label->setText(QApplication::translate("ControlSysUI", "Act_Max", 0));
        con_KD_label->setText(QApplication::translate("ControlSysUI", "Td", 0));
        con_KI_label->setText(QApplication::translate("ControlSysUI", "Ti", 0));
        Controlpara_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; text-decoration: underline;\">Control Parameters</span></p></body></html>", 0));
        con_KP_label->setText(QApplication::translate("ControlSysUI", "Kp", 0));
        tuneButton->setText(QApplication::translate("ControlSysUI", "&Tune!", 0));
        tuneButton->setShortcut(QApplication::translate("ControlSysUI", "Alt+T", 0));
        Fwd_label->setText(QApplication::translate("ControlSysUI", "Fwd", 0));
        fwd_check->setText(QString());
        Depth_label->setText(QApplication::translate("ControlSysUI", "Depth", 0));
        depth_check->setText(QString());
        yaw_label->setText(QApplication::translate("ControlSysUI", "Heading", 0));
        yaw_check->setText(QString());
        sm_check->setText(QString());
        sm_label->setText(QApplication::translate("ControlSysUI", "Side", 0));
        sendButton->setText(QApplication::translate("ControlSysUI", "S&end!", 0));
        sendButton->setShortcut(QApplication::translate("ControlSysUI", "Alt+S", 0));
        enabledButton->setText(QApplication::translate("ControlSysUI", "&Enable", 0));
        enabledButton->setShortcut(QApplication::translate("ControlSysUI", "Alt+E", 0));
        sm_label_2->setText(QApplication::translate("ControlSysUI", "Roll", 0));
        roll_check->setText(QString());
        sm_label_3->setText(QApplication::translate("ControlSysUI", "Pitch", 0));
        pitch_check->setText(QString());
        graphvalues->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p>Graph values:  x:       y:</p></body></html>", 0));
        fireButton->setText(QApplication::translate("ControlSysUI", "Fi&re", 0));
        fireButton->setShortcut(QApplication::translate("ControlSysUI", "Alt+R", 0));
        Goal_label->setText(QApplication::translate("ControlSysUI", "Goal", 0));
        DOF_label->setText(QApplication::translate("ControlSysUI", "DOF", 0));
        graphType->clear();
        graphType->insertItems(0, QStringList()
         << QApplication::translate("ControlSysUI", "Depth", 0)
         << QApplication::translate("ControlSysUI", "Heading", 0)
         << QApplication::translate("ControlSysUI", "Side", 0)
         << QApplication::translate("ControlSysUI", "Forward", 0)
         << QApplication::translate("ControlSysUI", "Roll", 0)
         << QApplication::translate("ControlSysUI", "Pitch", 0)
        );
        disableButton->setText(QApplication::translate("ControlSysUI", "&Disable", 0));
        disableButton->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+E", 0));
        refreshButton->setText(QApplication::translate("ControlSysUI", "&Refresh", 0));
        refreshButton->setShortcut(QApplication::translate("ControlSysUI", "Alt+R", 0));
        menuFile->setTitle(QApplication::translate("ControlSysUI", "&File", 0));
    } // retranslateUi

};

namespace Ui {
    class ControlSysUI: public Ui_ControlSysUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // TUNING_UI_H

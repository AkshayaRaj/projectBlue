/********************************************************************************
** Form generated from reading UI file 'vision.ui'
**
** Created by: Qt User Interface Compiler version 5.1.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef VISION_GUI_H
#define VISION_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Vision
{
public:
    QAction *actionOpen;
    QAction *actionSave;
    QAction *actionQuit;
    QWidget *centralwidget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *vboxLayout;
    QFrame *frontcam_2;
    QLabel *labelFront;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *_2;
    QFrame *frontcamfiltered;
    QLabel *labelFrontFiltered;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *_4;
    QFrame *bottomcam_2;
    QLabel *labelBottom;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *_3;
    QFrame *bottomcamfiltered;
    QLabel *labelBottomFiltered;
    QFrame *line;
    QComboBox *frontfilter;
    QComboBox *bottomfilter;
    QLabel *bottomcam;
    QLabel *frontcam;
    QFrame *line_2;
    QFrame *line_3;
    QComboBox *source_ddm;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *Vision)
    {
        if (Vision->objectName().isEmpty())
            Vision->setObjectName(QStringLiteral("Vision"));
        Vision->resize(765, 573);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Vision->sizePolicy().hasHeightForWidth());
        Vision->setSizePolicy(sizePolicy);
        Vision->setMinimumSize(QSize(341, 211));
        actionOpen = new QAction(Vision);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionSave = new QAction(Vision);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionQuit = new QAction(Vision);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        centralwidget = new QWidget(Vision);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(20, 50, 343, 213));
        vboxLayout = new QVBoxLayout(verticalLayoutWidget);
        vboxLayout->setObjectName(QStringLiteral("vboxLayout"));
        vboxLayout->setContentsMargins(0, 0, 0, 0);
        frontcam_2 = new QFrame(verticalLayoutWidget);
        frontcam_2->setObjectName(QStringLiteral("frontcam_2"));
        frontcam_2->setMinimumSize(QSize(0, 0));
        frontcam_2->setFrameShape(QFrame::StyledPanel);
        frontcam_2->setFrameShadow(QFrame::Raised);
        labelFront = new QLabel(frontcam_2);
        labelFront->setObjectName(QStringLiteral("labelFront"));
        labelFront->setGeometry(QRect(0, 0, 341, 211));
        labelFront->setMinimumSize(QSize(0, 0));

        vboxLayout->addWidget(frontcam_2);

        verticalLayoutWidget_2 = new QWidget(centralwidget);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(410, 50, 341, 211));
        _2 = new QVBoxLayout(verticalLayoutWidget_2);
        _2->setObjectName(QStringLiteral("_2"));
        _2->setContentsMargins(0, 0, 0, 0);
        frontcamfiltered = new QFrame(verticalLayoutWidget_2);
        frontcamfiltered->setObjectName(QStringLiteral("frontcamfiltered"));
        frontcamfiltered->setFrameShape(QFrame::StyledPanel);
        frontcamfiltered->setFrameShadow(QFrame::Raised);
        labelFrontFiltered = new QLabel(frontcamfiltered);
        labelFrontFiltered->setObjectName(QStringLiteral("labelFrontFiltered"));
        labelFrontFiltered->setGeometry(QRect(0, 0, 341, 211));

        _2->addWidget(frontcamfiltered);

        verticalLayoutWidget_3 = new QWidget(centralwidget);
        verticalLayoutWidget_3->setObjectName(QStringLiteral("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(20, 330, 331, 201));
        _4 = new QVBoxLayout(verticalLayoutWidget_3);
        _4->setObjectName(QStringLiteral("_4"));
        _4->setContentsMargins(0, 0, 0, 0);
        bottomcam_2 = new QFrame(verticalLayoutWidget_3);
        bottomcam_2->setObjectName(QStringLiteral("bottomcam_2"));
        bottomcam_2->setFrameShape(QFrame::StyledPanel);
        bottomcam_2->setFrameShadow(QFrame::Raised);
        labelBottom = new QLabel(bottomcam_2);
        labelBottom->setObjectName(QStringLiteral("labelBottom"));
        labelBottom->setGeometry(QRect(0, 0, 331, 201));

        _4->addWidget(bottomcam_2);

        verticalLayoutWidget_4 = new QWidget(centralwidget);
        verticalLayoutWidget_4->setObjectName(QStringLiteral("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(410, 330, 341, 201));
        _3 = new QVBoxLayout(verticalLayoutWidget_4);
        _3->setObjectName(QStringLiteral("_3"));
        _3->setContentsMargins(0, 0, 0, 0);
        bottomcamfiltered = new QFrame(verticalLayoutWidget_4);
        bottomcamfiltered->setObjectName(QStringLiteral("bottomcamfiltered"));
        bottomcamfiltered->setFrameShape(QFrame::StyledPanel);
        bottomcamfiltered->setFrameShadow(QFrame::Raised);
        labelBottomFiltered = new QLabel(bottomcamfiltered);
        labelBottomFiltered->setObjectName(QStringLiteral("labelBottomFiltered"));
        labelBottomFiltered->setGeometry(QRect(0, 0, 341, 201));

        _3->addWidget(bottomcamfiltered);

        line = new QFrame(centralwidget);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(-10, 260, 811, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        frontfilter = new QComboBox(centralwidget);
        frontfilter->setObjectName(QStringLiteral("frontfilter"));
        frontfilter->setGeometry(QRect(190, 10, 291, 27));
        bottomfilter = new QComboBox(centralwidget);
        bottomfilter->setObjectName(QStringLiteral("bottomfilter"));
        bottomfilter->setGeometry(QRect(200, 280, 291, 27));
        bottomcam = new QLabel(centralwidget);
        bottomcam->setObjectName(QStringLiteral("bottomcam"));
        bottomcam->setGeometry(QRect(20, 290, 131, 17));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        bottomcam->setFont(font);
        frontcam = new QLabel(centralwidget);
        frontcam->setObjectName(QStringLiteral("frontcam"));
        frontcam->setGeometry(QRect(20, 20, 131, 17));
        frontcam->setFont(font);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(370, 40, 20, 221));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralwidget);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(370, 320, 20, 251));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        source_ddm = new QComboBox(centralwidget);
        source_ddm->setObjectName(QStringLiteral("source_ddm"));
        source_ddm->setGeometry(QRect(610, 10, 141, 27));
        Vision->setCentralWidget(centralwidget);
        menubar = new QMenuBar(Vision);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 765, 25));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        Vision->setMenuBar(menubar);
        statusbar = new QStatusBar(Vision);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        Vision->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addSeparator();
        menuFile->addAction(actionSave);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);

        retranslateUi(Vision);
        QObject::connect(actionQuit, SIGNAL(triggered()), Vision, SLOT(close()));

        QMetaObject::connectSlotsByName(Vision);
    } // setupUi

    void retranslateUi(QMainWindow *Vision)
    {
        Vision->setWindowTitle(QApplication::translate("Vision", "Vision", 0));
        actionOpen->setText(QApplication::translate("Vision", "&Open", 0));
#ifndef QT_NO_TOOLTIP
        actionOpen->setToolTip(QApplication::translate("Vision", "Open a .bag file", 0));
#endif // QT_NO_TOOLTIP
        actionOpen->setShortcut(QApplication::translate("Vision", "Ctrl+O", 0));
        actionSave->setText(QApplication::translate("Vision", "&Save", 0));
#ifndef QT_NO_TOOLTIP
        actionSave->setToolTip(QApplication::translate("Vision", "Save a filtered image", 0));
#endif // QT_NO_TOOLTIP
        actionSave->setShortcut(QApplication::translate("Vision", "Ctrl+S", 0));
        actionQuit->setText(QApplication::translate("Vision", "&Quit", 0));
        actionQuit->setShortcut(QApplication::translate("Vision", "Ctrl+Q", 0));
        labelFront->setText(QApplication::translate("Vision", "Front Camera", 0));
        labelFrontFiltered->setText(QApplication::translate("Vision", "Front Camera", 0));
        labelBottom->setText(QApplication::translate("Vision", "Bottom Camera", 0));
        labelBottomFiltered->setText(QApplication::translate("Vision", "Bottom Camera", 0));
        bottomcam->setText(QApplication::translate("Vision", "Bottom Camera", 0));
        frontcam->setText(QApplication::translate("Vision", "Front Camera", 0));
        source_ddm->clear();
        source_ddm->insertItems(0, QStringList()
         << QApplication::translate("Vision", "Simulation", 0)
         << QApplication::translate("Vision", "Bag data", 0)
        );
        menuFile->setTitle(QApplication::translate("Vision", "&File", 0));
    } // retranslateUi

};

namespace Ui {
    class Vision: public Ui_Vision {};
} // namespace Ui

QT_END_NAMESPACE

#endif // VISION_GUI_H

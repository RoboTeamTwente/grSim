/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <QtGui>

#include <QApplication>
#include <QMenuBar>
#include <QGroupBox>
#include <QGridLayout>
#include <QSlider>
#include <QTimer>
#include <QToolBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QApplication>
#include <QDir>
#include <QClipboard>

#include <iostream>

#ifdef QT5
#include <QStatusBar>
#include <QMessageBox>
#endif

#include "mainwindow.h"

int MainWindow::getInterval()
{
    return ceil((1000.0f / configwidget->DesiredFPS()));
}

void MainWindow::customFPS(int fps)
{
    int k = ceil((1000.0f / fps));
    timer->setInterval(k);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QDir dir = qApp->applicationDirPath();
    dir.cdUp();
    current_dir = dir.path();
    /* Status Logger */

    /* Widgets */

    configwidget = new ConfigWidget();
    dockconfig = new ConfigDockWidget(this,configwidget);

    glwidget = new GLWidget(this,configwidget);
    glwidget->setWindowTitle(tr("Simulator"));
    glwidget->resize(512,512);

    visionServer = NULL;
    commandSocket = NULL;
    blueStatusSocket = NULL;
    yellowStatusSocket = NULL;
    reconnectVisionSocket();
    reconnectCommandSocket();
    reconnectBlueStatusSocket();
    reconnectYellowStatusSocket();

    glwidget->ssl->visionServer = visionServer;
    glwidget->ssl->commandSocket = commandSocket;
    glwidget->ssl->blueStatusSocket = blueStatusSocket;
    glwidget->ssl->yellowStatusSocket = yellowStatusSocket;

    /* Menus */

    QMenu *fileMenu = new QMenu("&File");
    menuBar()->addMenu(fileMenu);
    QAction *exit = new QAction("E&xit", fileMenu);
    exit->setShortcut(QKeySequence("Ctrl+X"));
    fileMenu->addAction(exit);


    glwidget->setWindowState(Qt::WindowMaximized);

    timer = new QTimer(this);
    timer->setInterval(getInterval());


    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    QObject::connect(exit, SIGNAL(triggered(bool)), this, SLOT(close()));
    QObject::connect(glwidget,SIGNAL(toggleFullScreen(bool)),this,SLOT(toggleFullScreen(bool)));
    QObject::connect(glwidget->ssl, SIGNAL(fpsChanged(int)), this, SLOT(customFPS(int)));
    //config related signals
    QObject::connect(configwidget->v_BallMass.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(changeBallMass()));
    QObject::connect(configwidget->v_BallBounce.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(changeBallGroundSurface()));
    QObject::connect(configwidget->v_BallBounceVel.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(changeBallGroundSurface()));
    QObject::connect(configwidget->v_BallFriction.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(changeBallGroundSurface()));
    QObject::connect(configwidget->v_BallSlip.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(changeBallGroundSurface()));
    QObject::connect(configwidget->v_BallAngularDamp.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(changeBallDamping()));
    QObject::connect(configwidget->v_BallLinearDamp.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(changeBallDamping()));
    QObject::connect(configwidget->v_Gravity.get(),  SIGNAL(wasEdited(VarPtr)), this, SLOT(changeGravity()));

    //geometry config vars
    QObject::connect(configwidget->v_DesiredFPS.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(changeTimer()));
    QObject::connect(configwidget->v_BallRadius.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_Field_Length.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_Field_Margin.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_Field_Width.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_Field_Penalty_Line.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_Field_Penalty_Point.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_Field_Penalty_Rad.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_Field_Rad.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_BlueTeam.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));
    QObject::connect(configwidget->v_YellowTeam.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(restartSimulator()));

    //network
    QObject::connect(configwidget->v_VisionMulticastAddr.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(reconnectVisionSocket()));
    QObject::connect(configwidget->v_VisionMulticastPort.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(reconnectVisionSocket()));
    QObject::connect(configwidget->v_CommandListenPort.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(reconnectCommandSocket()));
    QObject::connect(configwidget->v_BlueStatusSendPort.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(reconnectBlueStatusSocket()));
    QObject::connect(configwidget->v_YellowStatusSendPort.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(reconnectYellowStatusSocket()));
    timer->start();


    this->showMaximized();
    this->setWindowTitle("grSim");

    scene = new QGraphicsScene(0,0,800,600);
}

MainWindow::~MainWindow()
{
}

void MainWindow::changeCurrentRobot()
{
}

void MainWindow::changeCurrentTeam()
{
}

void MainWindow::changeGravity()
{
    dWorldSetGravity (glwidget->ssl->p->world,0,0,-configwidget->Gravity());
}

void MainWindow::changeTimer()
{
    timer->setInterval(getInterval());
}

QString dRealToStr(dReal a)
{
    QString s;
    s.setNum(a,'f',3);
    if (a>=0) return QString('+') + s;
    return s;
}

void MainWindow::update()
{
    if (glwidget->ssl->g->isGraphicsEnabled()) glwidget->updateGL();
    else glwidget->step();

    int R = robotIndex(glwidget->Current_robot,glwidget->Current_team);

    const dReal* vv = dBodyGetLinearVel(glwidget->ssl->robots[R]->chassis->body);
    static dVector3 lvv;
    dVector3 aa;
    aa[0]=(vv[0]-lvv[0])/configwidget->DeltaTime();
    aa[1]=(vv[1]-lvv[1])/configwidget->DeltaTime();
    aa[2]=(vv[2]-lvv[2])/configwidget->DeltaTime();

    lvv[0]=vv[0];
    lvv[1]=vv[1];
    lvv[2]=vv[2];
    QString ss;
    // fpslabel->setText(QString("Frame rate: %1 fps").arg(ss.sprintf("%06.2f",glwidget->getFPS())));
}


void MainWindow::changeBallMass()
{
    glwidget->ssl->ball->setMass(configwidget->BallMass());
}


void MainWindow::changeBallGroundSurface()
{
    PSurface* ballwithwall = glwidget->ssl->p->findSurface(glwidget->ssl->ball,glwidget->ssl->ground);
    ballwithwall->surface.mode = dContactBounce | dContactApprox1 | dContactSlip1 | dContactSlip2;
    ballwithwall->surface.mu = fric(configwidget->BallFriction());
    ballwithwall->surface.bounce = configwidget->BallBounce();
    ballwithwall->surface.bounce_vel = configwidget->BallBounceVel();
    ballwithwall->surface.slip1 = configwidget->BallSlip();
    ballwithwall->surface.slip2 = configwidget->BallSlip();
}

void MainWindow::changeBallDamping()
{
    dBodySetLinearDampingThreshold(glwidget->ssl->ball->body,0.001);
    dBodySetLinearDamping(glwidget->ssl->ball->body,configwidget->BallLinearDamp());
    dBodySetAngularDampingThreshold(glwidget->ssl->ball->body,0.001);
    dBodySetAngularDamping(glwidget->ssl->ball->body,configwidget->BallAngularDamp());
}

void MainWindow::restartSimulator()
{
    delete glwidget->ssl;
    glwidget->ssl = new SSLWorld(glwidget,glwidget->cfg,glwidget->forms[2],glwidget->forms[2]);
    glwidget->ssl->glinit();
    glwidget->ssl->visionServer = visionServer;
    glwidget->ssl->commandSocket = commandSocket;
    glwidget->ssl->blueStatusSocket = blueStatusSocket;
    glwidget->ssl->yellowStatusSocket = yellowStatusSocket;


}

void MainWindow::toggleFullScreen(bool a)
{
    if (a)
    {
        view = new GLWidgetGraphicsView(scene,glwidget);
        lastSize = glwidget->size();
        view->setViewport(glwidget);
        view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
        view->setFrameStyle(0);
        view->showFullScreen();
        view->setFocus();
        glwidget->fullScreen = true;
    }
    else {
        view->close();

        glwidget->show();
        glwidget->resize(lastSize);
        glwidget->fullScreen = false;
        glwidget->setFocusPolicy(Qt::StrongFocus);
        glwidget->setFocus();
    }
}

void MainWindow::setCurrentRobotPosition()
{
}


void MainWindow::reconnectBlueStatusSocket()
{
    if (blueStatusSocket!=NULL)
    {
        delete blueStatusSocket;
    }
    blueStatusSocket = new QUdpSocket(this);
    if (blueStatusSocket->bind(QHostAddress::Any,configwidget->BlueStatusSendPort()))
        std::cout << "Status send port binded for Blue Team on: " << configwidget->BlueStatusSendPort() << std::endl;
}

void MainWindow::reconnectYellowStatusSocket()
{
    if (yellowStatusSocket!=NULL)
    {
        delete yellowStatusSocket;
    }
    yellowStatusSocket = new QUdpSocket(this);
    if (yellowStatusSocket->bind(QHostAddress::Any,configwidget->YellowStatusSendPort()))
        std::cout << "Status send port binded for Yellow Team on: " << configwidget->YellowStatusSendPort() << std::endl;
}

void MainWindow::reconnectCommandSocket()
{
    if (commandSocket!=NULL)
    {
        QObject::disconnect(commandSocket,SIGNAL(readyRead()),this,SLOT(recvActions()));
        delete commandSocket;
    }

    commandSocket = new QUdpSocket(this);
    if (commandSocket->bind(QHostAddress::Any,configwidget->CommandListenPort()))
        std::cout << "Command listen port binded on: " << configwidget->CommandListenPort() << std::endl;

    QObject::connect(commandSocket,SIGNAL(readyRead()),this,SLOT(recvActions()));
}

void MainWindow::reconnectVisionSocket()
{
    if (visionServer == NULL) {
        visionServer = new RoboCupSSLServer(this);
    }
    visionServer->change_address(configwidget->VisionMulticastAddr());
    visionServer->change_port(configwidget->VisionMulticastPort());
}

void MainWindow::recvActions()
{
    glwidget->ssl->recvActions();
}

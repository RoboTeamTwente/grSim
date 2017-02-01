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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#if QT_VERSION >= 0x050000
#define QT5
#endif

#ifdef QT5
#include <QMdiArea>
#else
#include <QWorkspace>
#endif

#include <QLabel>

#include "glwidget.h"
#include "configwidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:
    void update();
    void changeCurrentRobot();
    void changeCurrentTeam();

    void changeBallMass();
    void changeBallGroundSurface();
    void changeBallDamping();
    void changeGravity();
    void changeTimer();

    void restartSimulator();
    void toggleFullScreen(bool);
    void setCurrentRobotPosition();

    void customFPS(int fps);
    void reconnectCommandSocket();
    void reconnectYellowStatusSocket();
    void reconnectBlueStatusSocket();
    void reconnectVisionSocket();
    void recvActions();
private:
    int getInterval();
    QTimer *timer;

    GLWidget *glwidget;
    ConfigWidget *configwidget;
    ConfigDockWidget *dockconfig;

    QString current_dir;

    QGraphicsScene *scene;
    GLWidgetGraphicsView *view;
    QSize lastSize;
    RoboCupSSLServer *visionServer;
    QUdpSocket *commandSocket;
    QUdpSocket *blueStatusSocket,*yellowStatusSocket;
};

#endif // MAINWINDOW_H

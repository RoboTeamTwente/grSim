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
#include "config.h"
#include <QtGui>

#include <QPainter>
#include "glwidget.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "logger.h"
#include <QLabel>

#include <iostream>

GLWidget::GLWidget(QWidget *parent,ConfigWidget* _cfg)
    : QGLWidget(parent)
{
    frames = 0;
    state = 0;
    first_time = true;
    cfg = _cfg;
    forms[0] = new RobotsFomation(-1);  //outside
    forms[1] = new RobotsFomation(-2);  //outside
    forms[2] = new RobotsFomation(1);  //inside type 1
    forms[3] = new RobotsFomation(2);  //inside type 2
    forms[4] = new RobotsFomation(3);  //inside type 1
    forms[5] = new RobotsFomation(4);  //inside type 2
    ssl = new SSLWorld(this,cfg,forms[2],forms[2]);
    Current_robot = 0;
    Current_team = 0;
    cammode = 0;
    setMouseTracking(true);

    setFocusPolicy(Qt::StrongFocus);
    fullScreen = false;
    ctrl = false;
    alt = false;
    kickingball = false;
    kickpower = 3.0;
    altTrigger = false;
    chipAngle = M_PI/4;
    chiping = false;
}

GLWidget::~GLWidget()
{
}

void GLWidget::moveRobot()
{
    ssl->show3DCursor = true;
    ssl->cursor_radius = cfg->robotSettings.RobotRadius;
    state = 1;
    moving_robot_id = clicked_robot;
}

void GLWidget::unselectRobot()
{
    ssl->show3DCursor = false;
    ssl->cursor_radius = cfg->robotSettings.RobotRadius;
    state = 0;
    moving_robot_id= robotIndex(Current_robot,Current_team);
}

void GLWidget::selectRobot()
{
    if (clicked_robot!=-1)
    {
        Current_robot = clicked_robot%ROBOT_COUNT;
        Current_team = clicked_robot/ROBOT_COUNT;
        emit selectedRobot();
    }
}

void GLWidget::resetRobot()
{
    if (Current_robot!=-1)
    {
        ssl->robots[robotIndex(Current_robot, Current_team)]->resetRobot();
    }
}

void GLWidget::switchRobotOnOff()
{
    int k = robotIndex(Current_robot, Current_team);
    if (Current_robot!=-1)
    {
        if (ssl->robots[k]->on==true)
        {
            ssl->robots[k]->on = false;
            emit robotTurnedOnOff(k,false);
        }
        else {
            ssl->robots[k]->on = true;
            emit robotTurnedOnOff(k,true);
        }
    }
}

void GLWidget::resetCurrentRobot()
{
    ssl->robots[robotIndex(Current_robot,Current_team)]->resetRobot();
}

void GLWidget::moveCurrentRobot()
{
    ssl->show3DCursor = true;
    ssl->cursor_radius = cfg->robotSettings.RobotRadius;
    state = 1;
    moving_robot_id = robotIndex(Current_robot,Current_team);
}

void GLWidget::moveBall()
{
    ssl->show3DCursor = true;
    ssl->cursor_radius = cfg->BallRadius();
    state = 2;
}

void GLWidget::update3DCursor(int mouse_x,int mouse_y)
{
    if (!ssl->g->isGraphicsEnabled()) return;
    ssl->updatedCursor = true;
    dVector3 xyz,hpr;
    dReal fx,fy,fz,rx,ry,rz,ux,uy,uz,px,py,pz;
    ssl->g->getViewpoint(xyz,hpr);
    ssl->g->getCameraForward(fx,fy,fz);
    ssl->g->getCameraRight(rx,ry,rz);
    ux = ry*fz - rz*fy;
    uy = rz*fx - rx*fz;
    uz = rx*fy - ry*fx;
    dReal w = width();
    dReal h = height();
    dReal xx,yy,z;
    dReal x = 1.0f - 2.0f*(dReal) mouse_x / w;
    dReal y = 1.0f - 2.0f*(dReal) mouse_y / h;
    ssl->g->getFrustum(xx,yy,z);
    x *= xx;
    y *= yy;
    px = -ux*y - rx*x - z*fx;
    py = -uy*y - ry*x - z*fy;
    pz = -uz*y - rz*x - z*fz;
    ssl->ray->setPose(xyz[0],xyz[1],xyz[2],px,py,pz);
}

dReal GLWidget::getFPS()
{
    return m_fps;
}


void GLWidget::initializeGL ()
{
    ssl->glinit();
}

void GLWidget::step()
{
    static double lastBallSpeed=-1;
    const dReal* ballV = dBodyGetLinearVel(ssl->ball->body);
    double ballSpeed = ballV[0]*ballV[0] + ballV[1]*ballV[1] + ballV[2]*ballV[2];
    ballSpeed  = sqrt(ballSpeed);
    lastBallSpeed = ballSpeed;
    rendertimer.restart();
    m_fps = frames /(time.elapsed()/1000.0);
    if (!(frames % ((int)(ceil(cfg->DesiredFPS()))))) {
        time.restart();
        frames = 0;
    }
    if (first_time) {ssl->step();first_time = false;}
    else {
        if (cfg->SyncWithGL())
        {
            dReal ddt=rendertimer.elapsed()/1000.0;
            if (ddt>0.05) ddt=0.05;
            ssl->step(ddt);
        }
        else {
            ssl->step(cfg->DeltaTime());
        }
    }
    frames ++;
}

void GLWidget::paintGL()
{
    if (!ssl->g->isGraphicsEnabled()) return;
    if (cammode==1)
    {
        dReal x,y,z;
        int R = robotIndex(Current_robot,Current_team);
        ssl->robots[R]->getXY(x,y);z = 0.3;
        ssl->g->setViewpoint(x,y,z,ssl->robots[R]->getDir(),-25,0);
    }
    if (cammode==-1)
    {
        dReal x,y,z;
        ssl->robots[lockedIndex]->getXY(x,y);z = 0.1;
        ssl->g->lookAt(x,y,z);
    }
    if (cammode==-2)
    {
        dReal x,y,z;
        ssl->ball->getBodyPosition(x,y,z);
        ssl->g->lookAt(x,y,z);
    }
    step();
    QFont font;
    for (int i=0;i<ROBOT_COUNT*2;i++)
    {
        dReal xx,yy;
        ssl->robots[i]->getXY(xx,yy);
        if (i>=ROBOT_COUNT) qglColor(Qt::yellow);
        else qglColor(Qt::cyan);
        renderText(xx,yy,0.3,QString::number(i%ROBOT_COUNT),font);
        if (!ssl->robots[i]->on){
            qglColor(Qt::red);
            font.setBold(true);
            renderText(xx,yy,0.4,"Off",font);
        }
        font.setBold(false);
    }
}

void GLWidget::changeCameraMode()
{
    static dReal xyz[3],hpr[3];
    if (cammode<0) cammode=0;
    else cammode ++;
    cammode %= 6;
    if (cammode==0)
        ssl->g->setViewpoint(0,-(cfg->Field_Width()+cfg->Field_Margin()*2.0f)/2.0f,3,90,-45,0);
    else if (cammode==1)
    {
        ssl->g->getViewpoint(xyz,hpr);
    }
    else if (cammode==2)
        ssl->g->setViewpoint(0,0,5,0,-90,0);
    else if (cammode==3)
        ssl->g->setViewpoint(0, (cfg->Field_Width()+cfg->Field_Margin()*2.0f)/2.0f,3,270,-45,0);
    else if (cammode==4)
        ssl->g->setViewpoint(-(cfg->Field_Length()+cfg->Field_Margin()*2.0f)/2.0f,0,3,0,-45,0);
    else if (cammode==5)
        ssl->g->setViewpoint((cfg->Field_Length()+cfg->Field_Margin()*2.0f)/2.0f,0,3,180,-45,0);
}

void GLWidget::putBall(dReal x,dReal y)
{
    ssl->ball->setBodyPosition(x,y,0.3);
    dBodySetLinearVel(ssl->ball->body,0,0,0);
    dBodySetAngularVel(ssl->ball->body,0,0,0);
}

void GLWidget::moveBallHere()
{
    ssl->ball->setBodyPosition(ssl->cursor_x,ssl->cursor_y,cfg->BallRadius()*2);
    dBodySetLinearVel(ssl->ball->body, 0.0, 0.0, 0.0);
    dBodySetAngularVel(ssl->ball->body, 0.0, 0.0, 0.0);

}

void GLWidget::lockCameraToRobot()
{
    cammode = -1;
    lockedIndex = robotIndex(Current_robot,Current_team);//clicked_robot;
}

void GLWidget::lockCameraToBall()
{
    cammode = -2;
}

void GLWidget::moveRobotHere()
{
    ssl->robots[robotIndex(Current_robot,Current_team)]->setXY(ssl->cursor_x,ssl->cursor_y);
    ssl->robots[robotIndex(Current_robot,Current_team)]->resetRobot();
}

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

#include "robot.h"
#include <iostream>

// ang2 = position angle
// ang  = rotation angle
Robot::Wheel::Wheel(Robot* robot, int _id, dReal ang, dReal ang2, int wheeltexid) {
    id = _id;
    rob = robot;
    dReal rad = rob->cfg->robotSettings.RobotRadius - rob->cfg->robotSettings.WheelThickness/2.0;
    ang *= M_PI/180.0f;
    ang2 *= M_PI/180.0f;
    dReal x = rob->m_x;
    dReal y = rob->m_y;
    dReal z = rob->m_z;
    dReal centerx = x + rad*cos(ang2);
    dReal centery = y + rad*sin(ang2);
    dReal centerz = z - rob->cfg->robotSettings.RobotHeight*0.5 + rob->cfg->robotSettings.WheelRadius
            - rob->cfg->robotSettings.BottomHeight;
    cyl = new PCylinder(centerx, centery, centerz, rob->cfg->robotSettings.WheelRadius,
            rob->cfg->robotSettings.WheelThickness, rob->cfg->robotSettings.WheelMass, 0.9, 0.9, 0.9, wheeltexid);
    cyl->setRotation(- sin(ang), cos(ang), 0, M_PI*0.5);
    cyl->setBodyRotation(- sin(ang), cos(ang), 0, M_PI*0.5, true);       //set local rotation matrix
    cyl->setBodyPosition(centerx - x, centery - y, centerz - z, true);       //set local position vector
    cyl->space = rob->space;

    rob->w->addObject(cyl);

    joint = dJointCreateHinge(rob->w->world, 0);

    dJointAttach(joint, rob->chassis->body, cyl->body);
    const dReal* a = dBodyGetPosition(cyl->body);
    dJointSetHingeAxis(joint, cos(ang), sin(ang), 0);
    dJointSetHingeAnchor(joint, a[0], a[1], a[2]);

    motor = dJointCreateAMotor(rob->w->world, 0);
    dJointAttach(motor, rob->chassis->body, cyl->body);
    dJointSetAMotorNumAxes(motor, 1);
    dJointSetAMotorAxis(motor, 0, 1, cos(ang), sin(ang), 0);
    dJointSetAMotorParam(motor, dParamFMax, rob->cfg->robotSettings.Wheel_Motor_FMax);
    speed = 0;
}

void Robot::Wheel::step() {
    dJointSetAMotorParam(motor, dParamVel, speed);
    dJointSetAMotorParam(motor, dParamFMax, rob->cfg->robotSettings.Wheel_Motor_FMax);
}

Robot::Kicker::Kicker(Robot* robot) {
    rob = robot;

    dReal x = rob->m_x;
    dReal y = rob->m_y;
    dReal z = rob->m_z;
    dReal centerx = x + (rob->cfg->robotSettings.RobotCenterFromKicker + rob->cfg->robotSettings.KickerThickness);
    dReal centery = y;
    dReal centerz = z - (rob->cfg->robotSettings.RobotHeight)*0.5f + rob->cfg->robotSettings.WheelRadius
            - rob->cfg->robotSettings.BottomHeight + rob->cfg->robotSettings.KickerZ;
    box = new PBox(centerx, centery, centerz, rob->cfg->robotSettings.KickerThickness,
            rob->cfg->robotSettings.KickerWidth, rob->cfg->robotSettings.KickerHeight,
            rob->cfg->robotSettings.KickerMass, 0.9, 0.9, 0.9);
    box->setBodyPosition(centerx - x, centery - y, centerz - z, true);
    box->space = rob->space;

    rob->w->addObject(box);

    joint = dJointCreateHinge(rob->w->world, 0);
    dJointAttach(joint, rob->chassis->body, box->body);
    const dReal* aa = dBodyGetPosition(box->body);
    dJointSetHingeAnchor(joint, aa[0], aa[1], aa[2]);
    dJointSetHingeAxis(joint, 0, - 1, 0);

    dJointSetHingeParam(joint, dParamVel, 0);
    dJointSetHingeParam(joint, dParamLoStop, 0);
    dJointSetHingeParam(joint, dParamHiStop, 0);

    rolling = 0;
    kicking = false;
    angle = 0;
}

void Robot::Kicker::step() {
    if (kicking) {
        box->setColor(1, 0.3, 0);
        kickstate --;
        if (kickstate <= 0) kicking = false;
    }
    else if (rolling != 0) {
        box->setColor(1, 0.7, 0);
        if (isTouchingBall()) {
            dReal fx, fy, fz;
            rob->chassis->getBodyDirection(fx, fy, fz);
            fz = sqrt(fx*fx + fy*fy);
            fx /= fz;
            fy /= fz;
            if (rolling == - 1) {
                fx = - fx;
                fy = - fy;
            }
            rob->getBall()->tag = rob->getID();

            dReal vx, vy, vz;
            dReal bx, by, bz;
            dReal kx, ky, kz;
            rob->chassis->getBodyDirection(vx, vy, vz);
            rob->getBall()->getBodyPosition(bx, by, bz);
            box->getBodyPosition(kx, ky, kz);
            dReal yy = - ((- (kx - bx)*vy + (ky - by)*vx))/rob->cfg->robotSettings.KickerWidth;
            //dReal dir = 1;
            //if (yy>0) dir = -1.0f;//never read
            dBodySetAngularVel(rob->getBall()->body, fy*rob->cfg->robotSettings.RollerTorqueFactor*1400,
                    - fx*rob->cfg->robotSettings.RollerTorqueFactor*1400, 0);
            //dBodyAddTorque(rob->getBall()->body,fy*rob->cfg->ROLLERTORQUEFACTOR(),-fx*rob->cfg->ROLLERTORQUEFACTOR(),0);
            dBodyAddTorque(rob->getBall()->body, yy*fx*rob->cfg->robotSettings.RollerPerpendicularTorqueFactor,
                    yy*fy*rob->cfg->robotSettings.RollerPerpendicularTorqueFactor, 0);
        }
    }
    else box->setColor(0.9, 0.9, 0.9);
}

bool Robot::Kicker::isTouchingBall() {
    dReal vx, vy, vz;
    dReal bx, by, bz;
    dReal kx, ky, kz;
    rob->chassis->getBodyDirection(vx, vy, vz);
    rob->getBall()->getBodyPosition(bx, by, bz);
    box->getBodyPosition(kx, ky, kz);
    kx += vx*rob->cfg->robotSettings.KickerThickness*0.5f;
    ky += vy*rob->cfg->robotSettings.KickerThickness*0.5f;
    dReal xx = fabs((kx - bx)*vx + (ky - by)*vy);
    dReal yy = fabs(- (kx - bx)*vy + (ky - by)*vx);
    dReal zz = fabs(kz - bz);
    return ((xx < rob->cfg->robotSettings.KickerThickness*2.0f + rob->cfg->BallRadius())
            && (yy < rob->cfg->robotSettings.KickerWidth*0.5f) && (zz < rob->cfg->robotSettings.KickerHeight*0.5f));
}

void Robot::Kicker::setRoller(int roller) {
    rolling = roller;
}

int Robot::Kicker::getRoller() {
    return rolling;
}

void Robot::Kicker::toggleRoller() {
    if (rolling == 0)
        rolling = 1;
    else rolling = 0;
}

void Robot::Kicker::rotate(dReal a) {
    angle += a;
}

void Robot::Kicker::rotateAbsolute(dReal a) {
    angle = a;
}

void Robot::Kicker::kick(dReal kickspeedx, dReal kickspeedz) {
    dReal dx, dy, dz;
    dReal vx, vy, vz;
    rob->chassis->getBodyDirection(dx, dy, dz);
    dz = 0;
    dReal zf = kickspeedz;

    // Rotate the direction of the robot with the angle of the kicker to get the directional vector of the ball
    dReal dxx, dyy, dzz;
    dxx = dx*cos(- angle) - dy*sin(- angle);
    dyy = dx*sin(- angle) + dy*cos(- angle);
    dzz = dz;

    if (isTouchingBall()) {
        dReal dlen = dxx*dxx + dyy*dyy + dzz*dzz;
        dlen = sqrt(dlen);
        vx = dxx*kickspeedx/dlen;
        vy = dyy*kickspeedx/dlen;
        vz = zf;
        const dReal* vball = dBodyGetLinearVel(rob->getBall()->body);
        dReal vn = - (vball[0]*dxx + vball[1]*dyy)*rob->cfg->robotSettings.KickerDampFactor;
        dReal vt = - (vball[0]*dyy - vball[1]*dxx);
        vx += vn*dxx - vt*dyy;
        vy += vn*dyy + vt*dxx;
        dBodySetLinearVel(rob->getBall()->body, vx, vy, vz);
    }
    kicking = true;
    kickstate = 10;
}

Robot::Robot(PWorld* world, PBall* ball, ConfigWidget* _cfg, dReal x, dReal y, dReal z, dReal r, dReal g, dReal b,
        int rob_id, int wheeltexid, int dir) {
    m_r = r;
    m_g = g;
    m_b = b;
    m_x = x;
    m_y = y;
    m_z = z;
    w = world;
    m_ball = ball;
    m_dir = dir;
    cfg = _cfg;
    m_rob_id = rob_id;
    prevYaw = 0;
    TH_switch = 0;
    prevAngleErr = 0;
    space = w->space;

    chassis = new PCylinder(x, y, z, cfg->robotSettings.RobotRadius, cfg->robotSettings.RobotHeight,
            cfg->robotSettings.BodyMass*0.99f, r, g, b, rob_id, true);
    chassis->space = space;
    w->addObject(chassis);

    dummy = new PBall(x, y, z, cfg->robotSettings.RobotCenterFromKicker, cfg->robotSettings.BodyMass*0.01f, 0, 0, 0);
    dummy->setVisibility(false);
    dummy->space = space;
    w->addObject(dummy);

    dummy_to_chassis = dJointCreateFixed(world->world, 0);
    dJointAttach(dummy_to_chassis, chassis->body, dummy->body);

    kicker = new Kicker(this);

    wheels[0] = new Wheel(this, 0, cfg->robotSettings.Wheel1Angle, cfg->robotSettings.Wheel1Angle, wheeltexid);
    wheels[1] = new Wheel(this, 1, cfg->robotSettings.Wheel2Angle, cfg->robotSettings.Wheel2Angle, wheeltexid);
    wheels[2] = new Wheel(this, 2, cfg->robotSettings.Wheel3Angle, cfg->robotSettings.Wheel3Angle, wheeltexid);
    wheels[3] = new Wheel(this, 3, cfg->robotSettings.Wheel4Angle, cfg->robotSettings.Wheel4Angle, wheeltexid);
    firsttime = true;
    on = true;
}

Robot::~Robot() {

}

PBall* Robot::getBall() {
    return m_ball;
}

int Robot::getID() {
    return m_rob_id - 1;
}

void normalizeVector(dReal &x, dReal &y, dReal &z) {
    dReal d = sqrt(x*x + y*y + z*z);
    x /= d;
    y /= d;
    z /= d;
}

void Robot::step() {
    if (on) {
        if (firsttime) {
            if (m_dir == - 1) setDir(180);
            firsttime = false;
        }
        wheels[0]->step();
        wheels[1]->step();
        wheels[2]->step();
        wheels[3]->step();
        kicker->step();
    }
    else {
        if (last_state) {
            wheels[0]->speed = wheels[1]->speed = wheels[2]->speed = wheels[3]->speed = 0;
            kicker->setRoller(0);
            wheels[0]->step();
            wheels[1]->step();
            wheels[2]->step();
            wheels[3]->step();
            kicker->step();
        }
    }
    last_state = on;
}

void Robot::drawLabel() {
    glPushMatrix();
    dVector3 pos;
    dReal fr_r, fr_b, fr_n;
    w->g->getFrustum(fr_r, fr_b, fr_n);
    const dReal txtWidth = 12.0f*fr_r/(dReal) w->g->getWidth();
    const dReal txtHeight = 24.0f*fr_b/(dReal) w->g->getHeight();
    pos[0] = dBodyGetPosition(chassis->body)[0];
    pos[1] = dBodyGetPosition(chassis->body)[1];
    pos[2] = dBodyGetPosition(chassis->body)[2];
    dReal xyz[3], hpr[3];
    w->g->getViewpoint(xyz, hpr);
    dReal ax = - pos[0] + xyz[0];
    dReal ay = - pos[1] + xyz[1];
    dReal az = - pos[2] + xyz[2];
    dReal fx, fy, fz;
    dReal rx, ry, rz;
    w->g->getCameraForward(fx, fy, fz);
    w->g->getCameraRight(rx, ry, rz);
    normalizeVector(fx, fy, fz);
    normalizeVector(rx, ry, rz);
    dReal zz = fx*ax + fy*ay + fz*az;
    dReal zfact = zz/fr_n;
    pos[2] += cfg->robotSettings.RobotHeight*0.5f + cfg->robotSettings.BottomHeight + cfg->robotSettings.WheelRadius
            + txtHeight*zfact;
    dMatrix3 rot;
    dRFromAxisAndAngle(rot, 0, 0, 0, 0);
    dReal tx = fy*rz - ry*fz;
    dReal ty = rx*fz - fx*rz;
    dReal tz = fx*ry - fy*rx;
    w->g->setTransform(pos, rot);
    w->g->useTexture((m_rob_id - 1) + 11 + 10*((on) ? 0 : 1));
    glShadeModel(GL_FLAT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_QUADS);
    glTexCoord2f(1, 1);
    glVertex3f(txtWidth*rx*zfact + txtHeight*tx*zfact, txtWidth*ry*zfact + txtHeight*ty*zfact,
            txtWidth*rz*zfact + txtHeight*tz*zfact);
    glTexCoord2f(0, 1);
    glVertex3f(- txtWidth*rx*zfact + txtHeight*tx*zfact, - txtWidth*ry*zfact + txtHeight*ty*zfact,
            - txtWidth*rz*zfact + txtHeight*tz*zfact);
    glTexCoord2f(0, 0);
    glVertex3f(- txtWidth*rx*zfact - txtHeight*tx*zfact, - txtWidth*ry*zfact - txtHeight*ty*zfact,
            - txtWidth*rz*zfact - txtHeight*tz*zfact);
    glTexCoord2f(1, 0);
    glVertex3f(txtWidth*rx*zfact - txtHeight*tx*zfact, txtWidth*ry*zfact - txtHeight*ty*zfact,
            txtWidth*rz*zfact - txtHeight*tz*zfact);
    glEnd();
    glDisable(GL_BLEND);
    w->g->noTexture();
    glPopMatrix();
}

void Robot::resetSpeeds() {
    wheels[0]->speed = wheels[1]->speed = wheels[2]->speed = wheels[3]->speed = 0;
}

void Robot::resetRobot() {
    resetSpeeds();
    dBodySetLinearVel(chassis->body, 0, 0, 0);
    dBodySetAngularVel(chassis->body, 0, 0, 0);
    dBodySetLinearVel(dummy->body, 0, 0, 0);
    dBodySetAngularVel(dummy->body, 0, 0, 0);
    dBodySetLinearVel(kicker->box->body, 0, 0, 0);
    dBodySetAngularVel(kicker->box->body, 0, 0, 0);
    for (int i = 0; i < 4; i ++) {
        dBodySetLinearVel(wheels[i]->cyl->body, 0, 0, 0);
        dBodySetAngularVel(wheels[i]->cyl->body, 0, 0, 0);
    }
    dReal x, y;
    getXY(x, y);
    setXY(x, y);
    if (m_dir == - 1) setDir(180);
    else setDir(0);
}

void Robot::getXY(dReal &x, dReal &y) {
    dReal xx, yy, zz;
    chassis->getBodyPosition(xx, yy, zz);
    x = xx;
    y = yy;
}

dReal Robot::getDir() {
    dReal x, y, z;
    chassis->getBodyDirection(x, y, z);
    dReal dot = x;//zarb dar (1.0,0.0,0.0)
    dReal length = sqrt(x*x + y*y);
    dReal absAng = (dReal) (acos((dReal) (dot/length))*(180.0f/M_PI));
    return (y > 0) ? absAng : - absAng;
}

void Robot::setXY(dReal x, dReal y) {
    dReal xx, yy, zz, kx, ky, kz;
    dReal height = ROBOT_START_Z(cfg);
    chassis->getBodyPosition(xx, yy, zz);
    chassis->setBodyPosition(x, y, height);
    dummy->setBodyPosition(x, y, height);
    kicker->box->getBodyPosition(kx, ky, kz);
    kicker->box->setBodyPosition(kx - xx + x, ky - yy + y, kz - zz + height);
    for (int i = 0; i < 4; i ++) {
        wheels[i]->cyl->getBodyPosition(kx, ky, kz);
        wheels[i]->cyl->setBodyPosition(kx - xx + x, ky - yy + y, kz - zz + height);
    }
}

void Robot::setDir(dReal ang) {
    ang *= M_PI/180.0f;
    chassis->setBodyRotation(0, 0, 1, ang);
    kicker->box->setBodyRotation(0, 0, 1, ang);
    dummy->setBodyRotation(0, 0, 1, ang);
    dMatrix3 wLocalRot, wRot, cRot;
    dVector3 localPos, finalPos, cPos;
    chassis->getBodyPosition(cPos[0], cPos[1], cPos[2], false);
    chassis->getBodyRotation(cRot, false);
    kicker->box->getBodyPosition(localPos[0], localPos[1], localPos[2], true);
    dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
    finalPos[0] += cPos[0];
    finalPos[1] += cPos[1];
    finalPos[2] += cPos[2];
    kicker->box->setBodyPosition(finalPos[0], finalPos[1], finalPos[2], false);
    for (int i = 0; i < 4; i ++) {
        wheels[i]->cyl->getBodyRotation(wLocalRot, true);
        dMultiply0(wRot, cRot, wLocalRot, 3, 3, 3);
        dBodySetRotation(wheels[i]->cyl->body, wRot);
        wheels[i]->cyl->getBodyPosition(localPos[0], localPos[1], localPos[2], true);
        dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
        finalPos[0] += cPos[0];
        finalPos[1] += cPos[1];
        finalPos[2] += cPos[2];
        wheels[i]->cyl->setBodyPosition(finalPos[0], finalPos[1], finalPos[2], false);
    }
}

void Robot::setSpeed(int i, dReal s) {
    if (! ((i >= 4) || (i < 0)))
        wheels[i]->speed = s;
}

void Robot::setSpeed(dReal vx, dReal vy, dReal vw) {
    // Calculate Motor Speeds
    dReal _DEG2RAD = M_PI/180.0;
    dReal motorAlpha[4] = {cfg->robotSettings.Wheel1Angle*_DEG2RAD, cfg->robotSettings.Wheel2Angle*_DEG2RAD,
                           cfg->robotSettings.Wheel3Angle*_DEG2RAD, cfg->robotSettings.Wheel4Angle*_DEG2RAD};

    dReal kP = 1.0;
    dReal kD = 0.2;
    for (unsigned int i = 0; i < 4; i ++) {
        dReal dw = (1.0/cfg->robotSettings.WheelRadius)*(
                   (cfg->robotSettings.RobotRadius*vw) -
                   (vx*sin(motorAlpha[i])) +
                   (vy*cos(motorAlpha[i])));

        double P = kP*dw;
        double D = - kD*(dw - getSpeed(i))/cfg->DesiredFPS();
        setSpeed(i, P + D);
    }
}

dReal Robot::getSpeed(int i) {
    if ((i >= 4) || (i < 0)) return - 1;
    return wheels[i]->speed;
}

void Robot::incSpeed(int i, dReal v) {
    if (! ((i >= 4) || (i < 0)))
        wheels[i]->speed += v;
}

//Implements our angle control cycle
void Robot::setAngle(dReal vx, dReal vy, dReal vw) {

//Get difference angle towards targetAngle
    double robotAngle = constrainAngle(getDir()*M_PI/180.0);
    double deltaAngle = constrainAngle(vw - robotAngle);
    if (deltaAngle < - M_PI)
        deltaAngle += 2*M_PI;
    else if (deltaAngle > M_PI)
        deltaAngle -= 2*M_PI;

//Get angular velocity
    double angularVel = robotAngle - prevYaw;
    prevYaw = robotAngle;

//Transform vx, vy relative to robot to newvx, newvy which are relative to the world
    double velocityAngle = atan2(vy, vx);
    velocityAngle -= robotAngle;
    double vLength = sqrt(vx*vx + vy*vy);
    double newvy = sin(velocityAngle)*vLength;
    double newvx = cos(velocityAngle)*vLength;

//PID control for the angle
    double tps = cfg->DesiredFPS();
    double kAngP = 6.0;
    double kAngD = 0.6;

    double pidAngP = kAngP*deltaAngle;
    double pidAngD = - kAngD*angularVel*tps;

    setSpeed(newvx, newvy, pidAngP + pidAngD);
}

std::vector<double> Robot::body2Wheels(dReal Fx, dReal Fy, dReal Fw) {
    float R = 0.0775; //robot radius
    float r = 0.0275; //wheel radius
    float cos60 = 0.5;
    float sin60 = 0.866;
    float PWM_CUTOFF = 3.0;
    float T_CUTOFF = (PWM_CUTOFF + 0.1F)*4*R/r;
    std::vector<double> output;
    if ((fabs(Fw) < T_CUTOFF) && (fabs(Fw) > T_CUTOFF/2 - 0.1F)) { //only using 2 motors for rotation, output doubled.
        std::cout << "Using only 2 wheels for rotation" << std::endl;
        output.push_back((1/sin60*Fx + 1/cos60*Fy + 2/R*Fw)*r/4);
        output.push_back((1/sin60*Fx - 1/cos60*Fy)*r/4);
        output.push_back((- 1/sin60*Fx - 1/cos60*Fy + 2/R*Fw)*r/4);
        output.push_back((- 1/sin60*Fx + 1/cos60*Fy)*r/4);
    }
    else { //NORMAL case
        output.push_back((1/sin60*Fx + 1/cos60*Fy + 1/R*Fw)*r/4);
        output.push_back((1/sin60*Fx - 1/cos60*Fy + 1/R*Fw)*r/4);
        output.push_back((- 1/sin60*Fx - 1/cos60*Fy + 1/R*Fw)*r/4);
        output.push_back((- 1/sin60*Fx + 1/cos60*Fy + 1/R*Fw)*r/4);
    }
    return output;
}

double Robot::scaleLimit(dReal Fx, dReal Fy, dReal Fw, dReal limit) {

    double scale;
    std::vector<double> imOutput = body2Wheels(Fx, Fy, Fw);
    double maxEl = fmax(fmax(fabs(imOutput[0]), fabs(imOutput[1])), fmax(fabs(imOutput[2]), fabs(imOutput[3])));

    if ((maxEl) > limit) {
        scale = limit/(maxEl);
    }
    else {
        scale = 1;
    }

    return scale;
}

std::vector<double> Robot::pwm2Motor(std::vector<double> power) {
    double PWM_CUTOFF = 3.0;
    double PWM_ROUNDUP = 3.1;
    double PWM_MAX = 100;
    for (int i = 0; i < 4; ++ i) {
        if (fabs(power[i]) < PWM_CUTOFF) { power[i] = 0.0F; }
        else if (fabs(power[i]) < PWM_ROUNDUP) {
            if (power[i] < 0) {
                power[i] = - PWM_ROUNDUP;
            }
            else {
                power[i] = PWM_ROUNDUP;
            }
        }
        else if (fabs(power[i]) > PWM_MAX) {
            if (power[i] < 0) {
                power[i] = - PWM_MAX;
            }
            else {
                power[i] = PWM_MAX;
            }
        }
    }
    std::cout << std::endl;
    std::vector<double> desiredVel;
    for (int j = 0; j < 4; ++ j) {
        desiredVel.push_back(374.0/60.0*power[j]*12/100*(1
                /0.288)); //power is % of total power given. 374 rpm/V *total Voltage (power*12/100). So current units is in rotation/second
    }
    return desiredVel;
}
double Robot::constrainAngle(double x) {
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
double Robot::angleControl(double angleRef, double yaw) {
    double R = 0.0775; //robot radius
    double r = 0.0275; //wheel radius
    double T_DIFF = 1/60.0; // integration for time. No clue how this is actually supposed to work on the robot.
    double angleErr = constrainAngle(angleRef - yaw);

    double dErr = constrainAngle(angleErr - prevAngleErr)/T_DIFF;
    prevAngleErr = angleErr;

    double output;

    TH_switch = - 1;

    output = angleErr*300.0 + dErr*13;
    double mag = fabs(output);
    double upper_lim = 300, deadzone = 0.03;
    double T_cutoff = 3.1*4*R/r;
    if (mag > upper_lim) {
        output = output/mag*upper_lim;
    }
    else if (fabs(angleErr) < deadzone + TH_switch*0.003) {
        output = 0;
        TH_switch = 1;
    }
    else if (mag < T_cutoff/2 && mag > 0.001F) {
        output = output/mag*T_cutoff/2.0;
    }
    return output;
}

void Robot::vectorRotate(double yaw, double* x, double* y) {
    double tempx = *x, tempy = *y;
    *x = cos(yaw)*tempx + sin(yaw)*tempy;
    *y = - sin(yaw)*tempx + cos(yaw)*tempy;
}
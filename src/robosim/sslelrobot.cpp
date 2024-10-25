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

#include "sslelrobot.h"

// ang2 = position angle
// ang  = rotation angle
SSLELRobot::Wheel::Wheel(SSLELRobot *robot, int _id, dReal ang, dReal ang2)
{
    this->id = _id;
    this->rob = robot;
    dReal rad = SSLELConfig::Robot().getRadius() + SSLELConfig::Robot().getWheelThickness() / 2.0;
    ang *= M_PI / 180.0f;
    ang2 *= M_PI / 180.0f;
    dReal x = this->rob->_x;
    dReal y = this->rob->_y;
    dReal z = this->rob->_z;
    dReal centerx = x + rad * cos(ang2);
    dReal centery = y + rad * sin(ang2);
    dReal centerz = z - SSLELConfig::Robot().getHeight() * 0.5 + SSLELConfig::Robot().getWheelRadius() - SSLELConfig::Robot().getBottomHeight();
    this->cyl = new PCylinder(centerx, centery, centerz, SSLELConfig::Robot().getWheelRadius(), SSLELConfig::Robot().getWheelThickness(), SSLELConfig::Robot().getWheelMass());
    this->cyl->setRotation(-sin(ang), cos(ang), 0, M_PI * 0.5);
    this->cyl->setBodyRotation(-sin(ang), cos(ang), 0, M_PI * 0.5, true);    //set local rotation matrix
    this->cyl->setBodyPosition(centerx - x, centery - y, centerz - z, true); //set local position vector

    this->rob->physics->addWheelObject(this->cyl);

    this->joint = dJointCreateHinge(this->rob->physics->world, nullptr);

    dJointAttach(this->joint, this->rob->chassis->body, this->cyl->body);
    const dReal *a = dBodyGetPosition(this->cyl->body);
    dJointSetHingeAxis(this->joint, cos(ang), sin(ang), 0);
    dJointSetHingeAnchor(this->joint, a[0], a[1], a[2]);

    this->motor = dJointCreateAMotor(this->rob->physics->world, nullptr);
    dJointAttach(this->motor, this->rob->chassis->body, this->cyl->body);
    dJointSetAMotorNumAxes(this->motor, 1);
    dJointSetAMotorAxis(this->motor, 0, 1, cos(ang), sin(ang), 0);
    dJointSetAMotorParam(this->motor, dParamFMax, SSLELConfig::Robot().getWheelMotorMaxTorque());
    this->maxAngularSpeed = (SSLELConfig::Robot().getWheelMotorMaxRPM() * 2 * M_PI) / 60;
    this->desiredAngularSpeed = 0;
}

SSLELRobot::Wheel::~Wheel() {
    delete this->cyl;
}

void SSLELRobot::Wheel::step()
{
    auto sent_speed = std::max(std::min(this->desiredAngularSpeed, this->maxAngularSpeed), -this->maxAngularSpeed);
    dJointSetAMotorParam(this->motor, dParamVel, sent_speed);
    dJointSetAMotorParam(this->motor, dParamFMax, SSLELConfig::Robot().getWheelMotorMaxTorque());
}

SSLELRobot::Kicker::Kicker(SSLELRobot* robot) : holdingBall(false)
{
    this->rob = robot;

    dReal x = this->rob->_x;
    dReal y = this->rob->_y;
    dReal z = this->rob->_z;

    dReal centerX = x + (SSLELConfig::Robot().getDistanceCenterKicker() - SSLELConfig::Robot().getKickerThickness()*0.5);
    dReal centerY = y;
    dReal centerZ = z - (SSLELConfig::Robot().getHeight()) * 0.5f  - SSLELConfig::Robot().getBottomHeight() + SSLELConfig::Robot().getKickerZ() + SSLELConfig::Robot().getKickerHeight() * 0.5;
    
    this->box = new PBox(
        centerX, centerY, centerZ, 
        SSLELConfig::Robot().getKickerThickness(), SSLELConfig::Robot().getKickerWidth(),
        SSLELConfig::Robot().getKickerHeight(),SSLELConfig::Robot().getKickerMass()
        );
    this->box->setBodyPosition(centerX - x, centerY - y, centerZ - z, true);

    this->rob->physics->addKickerObject(this->box);

    this->joint = dJointCreateHinge(this->rob->physics->world, 0);
    dJointAttach(this->joint,this->rob->chassis->body,this->box->body);
    
    const dReal *aa = dBodyGetPosition(this->box->body);
    dJointSetHingeAnchor(this->joint, aa[0], aa[1], aa[2]);
    dJointSetHingeAxis(this->joint, 0, -1,0);

    dJointSetHingeParam(this->joint,dParamVel,0);
    dJointSetHingeParam(this->joint,dParamLoStop,0);
    dJointSetHingeParam(this->joint,dParamHiStop,0);

    this->dribblerOn = false;
    this->kickerState = NO_KICK;
}

SSLELRobot::Kicker::~Kicker() {
    delete this->box;
}

void SSLELRobot::Kicker::step()
{
    if (!isTouchingBall() || !this->dribblerOn) unholdBall();
    if (this->kickerState != NO_KICK)
    {
        this->kickerCounter--;
        if (this->kickerCounter<=0) this->kickerState = NO_KICK;
    }
    else if (this->dribblerOn)
    {
        if (isTouchingBall())
        {
            holdBall();
        }
    }
}

bool SSLELRobot::Kicker::isTouchingBall()
{
    dReal vx,vy,vz;
    dReal bx,by,bz;
    dReal kx,ky,kz;

    this->rob->chassis->getBodyDirection(vx,vy,vz);
    this->rob->getBall()->getBodyPosition(bx,by,bz);
    this->box->getBodyPosition(kx,ky,kz);

    kx += vx * SSLELConfig::Robot().getKickerThickness()*0.5f;
    ky += vy * SSLELConfig::Robot().getKickerThickness()*0.5f;

    dReal xx = fabs((kx-bx)*vx + (ky-by)*vy);
    dReal yy = fabs(-(kx-bx)*vy + (ky-by)*vx);
    dReal zz = fabs(kz-bz);

    return ((xx < SSLELConfig::Robot().getKickerThickness() * 2.0f + SSLELConfig::World().getBallRadius()) && (yy < SSLELConfig::Robot().getKickerWidth()*0.5f) && (zz < SSLELConfig::Robot().getKickerHeight() * 0.5f));
}

KickStatus SSLELRobot::Kicker::getKickerStatus()
{
    return this->kickerState;
}

void SSLELRobot::Kicker::setDribbler(bool dribbler)
{
    this->dribblerOn = dribbler;
}

bool SSLELRobot::Kicker::getDribbler()
{
    return this->dribblerOn;
}

void SSLELRobot::Kicker::toggleDribbler()
{
    this->dribblerOn != this->dribblerOn;
}

void SSLELRobot::Kicker::kick(dReal kickSpeedX, dReal kickSpeedZ)
{    
    dReal dx,dy,dz;
    dReal vx,vy,vz;

    this->rob->chassis->getBodyDirection(dx,dy,dz);
    dz = 0;

    unholdBall();

    if (isTouchingBall())
    {
        dReal dlen = dx*dx + dy*dy + dz*dz;
        dlen = sqrt(dlen);

        vx = dx*kickSpeedX/dlen;
        vy = dy*kickSpeedX/dlen;
        vz = kickSpeedZ;

        const dReal* vball = dBodyGetLinearVel(rob->getBall()->body);
        dReal vn = -(vball[0]*dx + vball[1]*dy) * SSLELConfig::Robot().getKickerDampFactor();
        dReal vt = -(vball[0]*dy - vball[1]*dx);
        vx += vn * dx - vt * dy;
        vy += vn * dy + vt * dx;
        dBodySetLinearVel(this->rob->getBall()->body,vx,vy,vz);

        if (kickSpeedZ >= 1)
            this->kickerState = CHIP_KICK;
        else
            this->kickerState = FLAT_KICK;

        this->kickerCounter = 10;
    }
}

void SSLELRobot::Kicker::holdBall(){
    dReal vx,vy,vz;
    dReal bx,by,bz;
    dReal kx,ky,kz;

    this->rob->chassis->getBodyDirection(vx,vy,vz);
    this->rob->getBall()->getBodyPosition(bx,by,bz);
    this->box->getBodyPosition(kx,ky,kz);

    kx += vx * SSLELConfig::Robot().getKickerThickness()*0.5f;
    ky += vy * SSLELConfig::Robot().getKickerThickness()*0.5f;

    dReal xx = fabs((kx-bx)*vx + (ky-by)*vy);
    dReal yy = fabs(-(kx-bx)*vy + (ky-by)*vx);

    if(this->holdingBall || xx - SSLELConfig::World().getBallRadius() < 0) return;
    dBodySetLinearVel(this->rob->getBall()->body,0,0,0);
    this->robot_to_ball = dJointCreateHinge(this->rob->getWorld()->world, 0);
    dJointAttach(this->robot_to_ball, this->box->body, this->rob->getBall()->body);
    this->holdingBall = true;
}

void SSLELRobot::Kicker::unholdBall(){
    if(this->holdingBall) {
        dJointDestroy(this->robot_to_ball);
        this->holdingBall = false;
    }
}

SSLELRobot::SSLELRobot(PWorld *world, PBall *ball, dReal x, dReal y, dReal z,
               int robot_id, dReal dir)
{
    this->_x = x;
    this->_y = y;
    this->_z = z;
    this->physics = world;
    this->ball = ball;
    this->_dir = dir;
    this->rob_id = robot_id;

    this->chassis = new PCylinder(this->_x, this->_y, this->_z, SSLELConfig::Robot().getRadius(), SSLELConfig::Robot().getHeight(), SSLELConfig::Robot().getBodyMass());
    this->physics->addChassisObject(chassis);

    this->kicker = new Kicker(this);

    this->wheels[0] = new Wheel(this, 0, SSLELConfig::Robot().getWheel0Angle(), SSLELConfig::Robot().getWheel0Angle());
    this->wheels[1] = new Wheel(this, 1, SSLELConfig::Robot().getWheel1Angle(), SSLELConfig::Robot().getWheel1Angle());
    this->wheels[2] = new Wheel(this, 2, SSLELConfig::Robot().getWheel2Angle(), SSLELConfig::Robot().getWheel2Angle());
    this->wheels[3] = new Wheel(this, 3, SSLELConfig::Robot().getWheel3Angle(), SSLELConfig::Robot().getWheel3Angle());

    setDir(this->_dir);
}

SSLELRobot::~SSLELRobot() {
    delete this->chassis;
    delete this->kicker;
    for (auto &wheel : this->wheels) delete(wheel);
}

PBall *SSLELRobot::getBall()
{
    return this->ball;
}

PWorld *SSLELRobot::getWorld()
{
    return this->physics;
}

int SSLELRobot::getID()
{
    return this->rob_id - 1;
}

void SSLELRobot::step()
{
    for (auto &wheel : this->wheels)
        wheel->step();
    this->kicker->step();
}

void SSLELRobot::resetSpeeds()
{
    for (auto &wheel : this->wheels)
        wheel->desiredAngularSpeed = 0;
}

void SSLELRobot::resetRobot()
{
    resetSpeeds();
    dBodySetLinearVel(this->chassis->body, 0, 0, 0);
    dBodySetAngularVel(this->chassis->body, 0, 0, 0);
    for (auto &wheel : this->wheels)
    {
        dBodySetLinearVel(wheel->cyl->body, 0, 0, 0);
        dBodySetAngularVel(wheel->cyl->body, 0, 0, 0);
    }
    dReal x, y;
    getXY(x, y);
    setXY(x, y);
    setDir(this->_dir);
}

void SSLELRobot::getXY(dReal &x, dReal &y)
{
    dReal xx, yy, zz;
    this->chassis->getBodyPosition(xx, yy, zz);
    x = xx;
    y = yy;
}

dReal SSLELRobot::getDir(dReal &k)
{
    dReal x, y, z;
    this->chassis->getBodyDirection(x, y, z, k);

    dReal dot = x; //zarb dar (1.0,0.0,0.0)
    dReal length = sqrt(x * x + y * y);
    auto absAng = (dReal)(acos((dReal)(dot / length)) * (180.0f / M_PI));
    
    return (y > 0) ? absAng : 360-absAng;
}

void SSLELRobot::setXY(dReal x, dReal y)
{
    dReal xx, yy, zz, kx, ky, kz;
    dReal height = SSLEL_ROBOT_START_Z();
    this->chassis->getBodyPosition(xx, yy, zz);
    this->chassis->setBodyPosition(x, y, height);
    this->kicker->box->getBodyPosition(kx,ky,kz);
    this->kicker->box->setBodyPosition(kx-xx+x,ky-yy+y,kz-zz+height);

    for (auto &wheel : this->wheels)
    {
        wheel->cyl->getBodyPosition(kx, ky, kz);
        wheel->cyl->setBodyPosition(kx - xx + x, ky - yy + y, kz - zz + height);
    }
}

void SSLELRobot::setDir(dReal ang)
{
    dMatrix3 wLocalRot, wRot, cRot;
    dVector3 localPos, finalPos, cPos;
    ang *= M_PI / 180.0f;

    this->chassis->setBodyRotation(0, 0, 1, ang);
    this->kicker->box->setBodyRotation(0,0,1,ang);
    
    this->chassis->getBodyPosition(cPos[0], cPos[1], cPos[2], false);
    this->chassis->getBodyRotation(cRot, false);
    dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
    finalPos[0] += cPos[0];
    finalPos[1] += cPos[1];
    finalPos[2] += cPos[2];
    for (auto &wheel : this->wheels)
    {
        wheel->cyl->getBodyRotation(wLocalRot, true);
        dMultiply0(wRot, cRot, wLocalRot, 3, 3, 3);
        dBodySetRotation(wheel->cyl->body, wRot);
        wheel->cyl->getBodyPosition(localPos[0], localPos[1], localPos[2], true);
        dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
        finalPos[0] += cPos[0];
        finalPos[1] += cPos[1];
        finalPos[2] += cPos[2];
        wheel->cyl->setBodyPosition(finalPos[0], finalPos[1], finalPos[2], false);
    }
}

void SSLELRobot::setWheelDesiredAngularSpeed(int i, dReal s)
{
    if (!((i >= 4) || (i < 0)))
        this->wheels[i]->desiredAngularSpeed = s;
}

void SSLELRobot::setDesiredSpeedLocal(dReal vx, dReal vy, dReal vw)
{
    // Calculate Motor Speeds
    dReal _DEG2RAD = M_PI / 180.0;
    dReal motorAlpha[4] = {SSLELConfig::Robot().getWheel0Angle() * _DEG2RAD, SSLELConfig::Robot().getWheel1Angle() * _DEG2RAD, SSLELConfig::Robot().getWheel2Angle() * _DEG2RAD, SSLELConfig::Robot().getWheel3Angle() * _DEG2RAD};

    // Convert local robot speed to rad/s
    dReal dw0 =  (1.0 / SSLELConfig::Robot().getWheelRadius()) * (( (SSLELConfig::Robot().getRadius() * vw) - (vx * sin(motorAlpha[0])) + (vy * cos(motorAlpha[0]))) );
    dReal dw1 =  (1.0 / SSLELConfig::Robot().getWheelRadius()) * (( (SSLELConfig::Robot().getRadius() * vw) - (vx * sin(motorAlpha[1])) + (vy * cos(motorAlpha[1]))) );
    dReal dw2 =  (1.0 / SSLELConfig::Robot().getWheelRadius()) * (( (SSLELConfig::Robot().getRadius() * vw) - (vx * sin(motorAlpha[2])) + (vy * cos(motorAlpha[2]))) );
    dReal dw3 =  (1.0 / SSLELConfig::Robot().getWheelRadius()) * (( (SSLELConfig::Robot().getRadius() * vw) - (vx * sin(motorAlpha[3])) + (vy * cos(motorAlpha[3]))) );

    setWheelDesiredAngularSpeed(0 , dw0);
    setWheelDesiredAngularSpeed(1 , dw1);
    setWheelDesiredAngularSpeed(2 , dw2);
    setWheelDesiredAngularSpeed(3 , dw3);
}

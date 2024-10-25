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

#ifndef SSLELROBOT_H
#define SSLELROBOT_H

#include "physics/pworld.h"
#include "physics/pcylinder.h"
#include "physics/pbox.h"
#include "physics/pball.h"
#include "sslelconfig.h"
#include "kickstatus.h"

class SSLELRobot
{
    PWorld *physics;
    PBall *ball;
    int rob_id;

public:
    dReal _x, _y, _z;
    dReal _dir;
    dSpaceID space;
    PObject *chassis;

    class Wheel
    {
    public:
        int id;
        Wheel(SSLELRobot *robot, int _id, dReal ang, dReal ang2);
        ~Wheel();
        void step();
        dJointID joint;
        dJointID motor;
        PCylinder *cyl;
        dReal desiredAngularSpeed, maxAngularSpeed; // rad/s
        SSLELRobot *rob;
    } * wheels[4]{};
    class Kicker
    {
      private:
        KickStatus kickerState;
        bool dribblerOn;
        int kickerCounter;
        bool holdingBall;
      public:
        Kicker(SSLELRobot* robot);
        ~Kicker();
        void step();
        void kick(dReal kickSpeedX, dReal kickSpeedZ);
        void setDribbler(bool dribbler);
        bool getDribbler();
        void toggleDribbler();
        bool isTouchingBall();
        KickStatus getKickerStatus();
        void holdBall();
        void unholdBall();
        dJointID joint;
        dJointID robot_to_ball;
        PBox* box;
        SSLELRobot* rob;
    } *kicker;

    SSLELRobot(PWorld *world, PBall *ball, dReal x, dReal y, dReal z,
           int robot_id, dReal dir);
    ~SSLELRobot();
    void step();
    void setDesiredSpeedLocal(dReal vx, dReal vy, dReal vw);
    void setWheelDesiredAngularSpeed(int i, dReal s); //i = 0,1,2,3
    void setSpeed(dReal vx, dReal vy, dReal vw);
    void incSpeed(int i, dReal v);
    void resetSpeeds();
    void resetRobot();
    void getXY(dReal &x, dReal &y);
    dReal getDir(dReal &k);
    void setXY(dReal x, dReal y);
    void setDir(dReal ang);
    int getID();
    PBall *getBall();
    PWorld *getWorld();
};

#define SSLEL_ROBOT_START_Z() (SSLELConfig::Robot().getHeight() * 0.5 + SSLELConfig::Robot().getBottomHeight())

#endif // SSLELROBOT_H

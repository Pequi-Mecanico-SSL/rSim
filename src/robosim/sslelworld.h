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

#ifndef SSLELWORLD_H
#define SSLELWORLD_H


#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"

#include "sslelrobot.h"
#include "utils.h"
#include <unordered_map>

#define SSLEL_WALL_COUNT 10
#define SSLEL_MAX_ROBOT_COUNT 6 //don't change

class SSLELWorld
{
private:
    double timeStep;
    int stateSize;
    std::vector<double> state;

public:
    bool fullSpeed = false;
    PWorld *physics;
    PBall *ball;
    PGround *ground;
    PFixedBox *walls[SSLEL_WALL_COUNT]{};
    SSLELRobot *robots[SSLEL_MAX_ROBOT_COUNT * 2]{};
    SSLELConfig::Field field = SSLELConfig::Field();

    SSLELWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, double timeStep,
             std::vector<double> ballPos, std::vector<std::vector<double>> blueRobotsPos, std::vector<std::vector<double>> yellowRobotsPos);
    ~SSLELWorld();
    void step(std::vector<std::vector<double>> actions);
    void replace(std::vector<double> ballPos, std::vector<std::vector<double>> blueRobotsPos, std::vector<std::vector<double>> yellowRobotsPos);
    void initWalls();
    int getNumRobotsBlue() { return this->field.getRobotsBlueCount(); }
    int getNumRobotsYellow() { return this->field.getRobotsYellowCount(); }
    const std::unordered_map<std::string, double> getFieldParams();
    const std::vector<double> &getState();
    void setActions(std::vector<std::vector<double>> actions);
};

#endif // SSLELWorld_H

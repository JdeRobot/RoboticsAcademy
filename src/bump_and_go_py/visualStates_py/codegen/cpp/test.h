/*
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

*/
#ifndef TEST_H
#define TEST_H

#include "state.h"
#include "temporaltransition.h"
#include "conditionaltransition.h"
#include "interfaces.h"

#include <jderobot/motors.h>

class State0 : public State {
public:
    State0(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):State(id, initial, interfaces, cycleDuration, parent, gui) {}
    virtual void runCode();
};

class State1 : public State {
public:
    State1(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):State(id, initial, interfaces, cycleDuration, parent, gui) {}
    virtual void runCode();
};

class State2 : public State {
public:
    State2(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):State(id, initial, interfaces, cycleDuration, parent, gui) {}
    virtual void runCode();
};

class Tran0 : public ConditionalTransition {
public:
    Tran0(int id, int destId, Interfaces* interfaces):ConditionalTransition(id, destId, interfaces) {}
    virtual void init();
    virtual bool checkCondition();
    virtual void runCode();
};

class MyInterfaces : public Interfaces {
public:
    jderobot::MotorsPrx myMotors;

    virtual void connectProxies(int argc, char* argv[]);
    virtual void destroyProxies();

};

#endif
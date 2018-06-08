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
#ifndef STATE_H
#define STATE_H

#include <vector>
#include <map>
#include "transition.h"
#include "runtimegui.h"

class State {
protected:
    int id;
    bool active;
    bool running;
    State* parent;
    State* currentState;
    bool initial;
    bool displayGui;
    int cycleDuration;

    std::vector<State*> states;
    std::vector<Transition*> transitions;
    std::map<int, State*> statesById;

    RunTimeGui* gui;
    pthread_t thread;

public:
    State(int id, bool initial, int cycleDuration, State* parent, RunTimeGui* gui);

    void init();
    void addState(State* state);
    void addTransition(Transition* transition);
    void startThread();
    void run();
    void stop();
    void join();
    long getCurrentTime();

    virtual void runCode() {}

    static void* threadRunner(void*);

};

#endif
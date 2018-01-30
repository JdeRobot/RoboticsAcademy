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
#include "test.h"
#include <iostream>
#include <easyiceconfig/EasyIce.h>
#include <string>
#include "runtimegui.h"

void State0::runCode() {
    std::cout << " runcode for state0" << std::endl;
}

void State1::runCode() {
    std::cout << " runcode for state1" << std::endl;
}

void State2::runCode() {
    std::cout << " runcode for state2" << std::endl;
}

void Tran0::init() {
    std::cout << " tran1 init" << std::endl;
}

bool Tran0::checkCondition() {
    std::cout << " tran1 checkcondition" << std::endl;
    return false;
}

void Tran0::runCode() {
    std::cout << " tran1 runcode" << std::endl;
}

void MyInterfaces::connectProxies(int argc, char* argv[]) {
    ice = EasyIce::initialize(argc, argv);

    Ice::ObjectPrx tempmyMotors = ice->propertyToProxy("automata.myMotors.Proxy");
    if (tempmyMotors == 0) {
        throw "cannot create proxy from automata.myMotors.Proxy";
    }
    myMotors = jderobot::MotorsPrx::checkedCast(tempmyMotors);
    if (myMotors == 0) {
        throw "invalid proxy automata.myMotors.Proxy";
    }
    std::cout << "myMotors connected" << std::endl;
}

void MyInterfaces::destroyProxies() {
    if (ice != 0) {
        ice->destroy();
    }
}

pthread_t guiThread;
RunTimeGui* runTimeGui = NULL;
bool displayGui = false;

void readArgs(int *argc, char* argv[]) {
    int i;
    std::string splitedArg;

    for(i = 0; i < *argc; i++){
        splitedArg = strtok(argv[i], "=");
        if (splitedArg.compare("--displaygui") == 0){
            splitedArg = strtok(NULL, "=");
            if (splitedArg.compare("true") == 0 || splitedArg.compare("True") == 0){
                displayGui = true;
                std::cout << "displayGui ENABLED" << std::endl;
            }else{
                displayGui = false;
                std::cout << "displayGui DISABLED" << std::endl;
            }
        }
        if(i == *argc -1){
            (*argc)--;
        }
    }
}

void* runGui(void*) {
    runTimeGui = new RunTimeGui();
}

int main (int argc, char* argv[]) {
    MyInterfaces interfaces;
    try {
        interfaces.connectProxies(argc, argv);
    } catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        interfaces.destroyProxies();
        return 1;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        interfaces.destroyProxies();
        return 1;
    }

    readArgs(&argc, argv);
    displayGui = true;
    std::cout << "displaygui:" << displayGui << std::endl;

    if (displayGui) {
        pthread_create(&guiThread, NULL, &runGui, NULL);
        while (runTimeGui == NULL) {
            usleep(10000);
        }

        runTimeGui->addState(0, "root", true, 0.0, 0.0, -1);
        runTimeGui->addState(1, "state1", true, 0.0, 0.0, 0);
        runTimeGui->addState(2, "state2", false, 0.0, 0.0, 0);

        runTimeGui->emitActiveStateById(0);
        runTimeGui->emitLoadFromRoot();
    }

    // create transitions
    State* state0 = new State0(0, true, (Interfaces*)&interfaces, 100, NULL, runTimeGui);

    State* state1 = new State1(1, true, (Interfaces*)&interfaces, 100, state0, runTimeGui);
    State* state2 = new State2(2, false, (Interfaces*)&interfaces, 100, state0, runTimeGui);

    // create state transitions
    Transition* tran0 = new Tran0(0, 2, (Interfaces*)&interfaces);
    state1->addTransition(tran0);

    std::cout << "just before state thread" << std::endl;

    state0->startThread();
    std::cout << "thread started" << std::endl;
    state0->join();
    std::cout << "thread joined" << std::endl;

}
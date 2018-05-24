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
#include "temporaltransition.h"
#include <sys/time.h>

TemporalTransition::TemporalTransition(int id, int destinationId, int elapsedTime):
        Transition(id, destinationId) {
    this->elapsedTime = elapsedTime;
}

void TemporalTransition::init() {
    startTime = getCurrentTime();
}

bool TemporalTransition::checkCondition() {
    long diffTime = getCurrentTime()-startTime;
    if (diffTime > elapsedTime) {
        return true;
    } else {
        return false;
    }
}

long TemporalTransition::getCurrentTime() {
    struct timeval a;
    gettimeofday(&a, 0);
    return (a.tv_sec * 1000000 + a.tv_usec)/1000;
}

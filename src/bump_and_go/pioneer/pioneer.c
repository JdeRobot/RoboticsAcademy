/*
 *
 *  Copyright (C) 1997-2008 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *
 *  Authors : Jose María Cañas Plaza <jmplaza@gsyc.es>
 *
 */


#include "pioneer.h"
#include <math.h>

/* sensor positions in the Robot FrameOfReference */
float laser_coord[5];
float us_coord[NUM_SONARS][5];
/**< Estructura para poder cambiar medidas de sensor a coordenadas locales al robot, y de estas al sist ref inicial: xsensor, ysensor, orientsensor,cossensor y sinsensor del sensor respecto del sistema solidario con el robot. Es fija. */
float camera_coord[5];

/** coordinate transformations from one FrameOfReference to another. */
void us2xy(int numsensor, float d, float phi, Tvoxel *point, float *jderobot)

/*  Calcula la posicion respecto de sistema de referencia inicial (sistema odometrico) del punto detectado en el sistema de coordenadas solidario al sensor. OJO depende de estructura posiciones y de por el sensor, sabiendo que:
a) el robot se encuentra en robot[0], robot[1] con orientacion robot[2] respecto al sistema de referencia externo,
b) que el sensor se encuentra en xsen, ysen con orientacion asen respecto del sistema centrado en el robot apuntando hacia su frente, 
c) el punto esta a distancia d del sensor en el angulo phi 
 */ {
    float Xp_sensor, Yp_sensor, Xp_robot, Yp_robot;

    Xp_sensor = d * cos(DEGTORAD * phi);
    Yp_sensor = d * sin(DEGTORAD * phi);
    /* Coordenadas del punto detectado por el US con respecto al sistema del sensor, eje x+ normal al sensor */
    Xp_robot = us_coord[numsensor][0] + Xp_sensor * us_coord[numsensor][3] - Yp_sensor * us_coord[numsensor][4];
    Yp_robot = us_coord[numsensor][1] + Yp_sensor * us_coord[numsensor][3] + Xp_sensor * us_coord[numsensor][4];
    /* Coordenadas del punto detectado por el US con respecto al robot */
    (*point).x = Xp_robot * jderobot[3] - Yp_robot * jderobot[4] + jderobot[0];
    (*point).y = Yp_robot * jderobot[3] + Xp_robot * jderobot[4] + jderobot[1];
    /* Coordenadas del punto con respecto al origen del SdeR */
}

void laser2xy(int reading, float d, Tvoxel *point, float *jderobot)

/*  Calcula la posicion respecto de sistema de referencia inicial (sistema odometrico) del punto detectado en el sistema de coordenadas solidario al sensor. OJO depende de estructura posiciones y de por el sensor, sabiendo que:
a) el robot se encuentra en robot[0], robot[1] con orientacion robot[2] respecto al sistema de referencia externo,
b) que el sensor se encuentra en xsen, ysen con orientacion asen respecto del sistema centrado en el robot apuntando hacia su frente, 
 */ {
    float Xp_sensor, Yp_sensor, Xp_robot, Yp_robot, phi;

    phi = -90. + 180. * reading / NUM_LASER;
    Xp_sensor = d * cos(DEGTORAD * phi);
    Yp_sensor = d * sin(DEGTORAD * phi);
    Xp_robot = laser_coord[0] + Xp_sensor * laser_coord[3] - Yp_sensor * laser_coord[4];
    Yp_robot = laser_coord[1] + Yp_sensor * laser_coord[3] + Xp_sensor * laser_coord[4];
    /* Coordenadas del punto detectado por el laser con respecto al robot */
    (*point).x = Xp_robot * jderobot[3] - Yp_robot * jderobot[4] + jderobot[0];
    (*point).y = Yp_robot * jderobot[3] + Xp_robot * jderobot[4] + jderobot[1];
    /* Coordenadas del punto con respecto al origen del SdeR */
}

/* init the pioneer robot */
void init_pioneer() {
    int j;

    /*  Posicion y orientacion de los sensores con respecto al centro del eje trasero del robot. Dado un sistema de coordenadas con la X en la direccion de movimiento del robot, los angulos se miden considerando que el eje X toma valor 0 y siendo positivos cuando se gira en sentido contrario al de movimiento de las agujas del reloj. Se utiliza para cambiar las distancias sensoriales al sistema de referencia local, solidario con el robot-enclosure. la rellena con milimetros y grados.   */
    us_coord[0][0] = 115.;
    us_coord[0][1] = 130.;
    us_coord[0][2] = 90.;
    us_coord[1][0] = 155.;
    us_coord[1][1] = 115.;
    us_coord[1][2] = 50.;
    us_coord[2][0] = 190.;
    us_coord[2][1] = 80.;
    us_coord[2][2] = 30.;
    us_coord[3][0] = 210.;
    us_coord[3][1] = 25.;
    us_coord[3][2] = 10.;
    us_coord[4][0] = 210.;
    us_coord[4][1] = -25.;
    us_coord[4][2] = 350.;
    us_coord[5][0] = 190.;
    us_coord[5][1] = -80.;
    us_coord[5][2] = 330.;
    us_coord[6][0] = 155.;
    us_coord[6][1] = -115.;
    us_coord[6][2] = 310;
    us_coord[7][0] = 115.;
    us_coord[7][1] = -130.;
    us_coord[7][2] = 270.;
    us_coord[8][0] = -115.;
    us_coord[8][1] = -130.;
    us_coord[8][2] = 270.;
    us_coord[9][0] = -155.;
    us_coord[9][1] = -115.;
    us_coord[9][2] = 230.;
    us_coord[10][0] = -190.;
    us_coord[10][1] = -80.;
    us_coord[10][2] = 210.;
    us_coord[11][0] = -210.;
    us_coord[11][1] = -25.;
    us_coord[11][2] = 190.;
    us_coord[12][0] = -210.;
    us_coord[12][1] = 25.;
    us_coord[12][2] = 170.;
    us_coord[13][0] = -190.;
    us_coord[13][1] = 80.;
    us_coord[13][2] = 150.;
    us_coord[14][0] = -155.;
    us_coord[14][1] = 115.;
    us_coord[14][2] = 130.;
    us_coord[15][0] = -115.;
    us_coord[15][1] = 130.;
    us_coord[15][2] = 90.;

    for (j = 0; j < NUM_SONARS; j++) {
        us_coord[j][3] = cos(us_coord[j][2] * DEGTORAD);
        us_coord[j][4] = sin(us_coord[j][2] * DEGTORAD);
    }

    laser_coord[0] = 19.;
    laser_coord[1] = 0.;
    laser_coord[2] = 0.;
    laser_coord[3] = cos(laser_coord[2] * DEGTORAD);
    laser_coord[4] = sin(laser_coord[2] * DEGTORAD);

    camera_coord[0] = 190.;
    camera_coord[1] = 0.;
    camera_coord[2] = 0.;
    camera_coord[3] = cos(camera_coord[2] * DEGTORAD);
    camera_coord[4] = sin(camera_coord[2] * DEGTORAD);
}

/*
 *  Copyright (C) 1997-2010 JDE Developers Team
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
 *  Authors : Jose María Cañas <jmplaza@gsyc.es>
 *            Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
 */

#ifndef BODY_ICE
#define BODY_ICE

#include </home/rperez/TFG/3DViewer-web/test_server/common.ice>

module jderobot{

	/**
	* PCL
	*/
	struct RGBPoint{
      float x;
      float y;
      float z;
      float r;
      float g;
      float b;
	  int id;
	};

	struct Point{
	    float x;
	    float y;
	    float z;
	};

	struct Segment{
	    Point fromPoint;
	    Point toPoint;
	};

};
#endif

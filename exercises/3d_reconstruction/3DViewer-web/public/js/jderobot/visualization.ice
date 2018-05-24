/*
 *  Copyright (C) 1997-2016 JDE Developers Team
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
 *  Authors : Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
 */

#ifndef VISUALIZATION_ICE
#define VISUALIZATION_ICE

#include </home/rperez/TFG/3DViewer-web/test_server/common.ice>
#include </home/rperez/TFG/3DViewer-web/test_server/primitives.ice>


module jderobot{

	struct Color{
	    float r;
	    float g;
	    float b;
	};
	
	sequence<RGBPoint> bufferPoint; 
	sequence<Segment> bufferSegment;
  /**
   * Interface to the Visualization interaction.
   */
	interface Visualization
	{
        void drawSegment(Segment seg, Color c);
	void getSegment (out bufferSegment bseg, out Color c);
        void drawPoint(Point p, Color c);
	bufferPoint getPoints();
        void clearAll();
	};




};
#endif

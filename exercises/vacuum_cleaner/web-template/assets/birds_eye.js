// Retreive the canvas elements and context
var mapCanvas = document.getElementById("birds-eye"),
	ctx = mapCanvas.getContext("2d");
	
var trail = [],
	coords = [-1, -1];;

// Complete draw function
function draw(x, y, ax, ay){
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	

	drawTrail(coords[0], coords[1]);
	coords = drawTriangle(x, y, ax, ay);
}

// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points
function drawCircle(x, y){
	cursor_x = x;
	cursor_y = y;
	
	ctx.beginPath();
	ctx.arc(x, y, 2, 0, 2 * Math.PI);
	ctx.closePath();
	
	ctx.lineWidth = 1.5;
	ctx.strokeStyle = "#0000FF";
	ctx.stroke();
	
	ctx.fillStyle = "#0000FF";
	ctx.fill();
}

// Testing to be carried out with Python interface
function drawTriangle(posx, posy, angx, angy){	
	ctx.beginPath();
	
	px = posx;
	py = posy;
	
	// The main line
	ctx.strokeStyle = '#FF0000';
	//ctx.moveTo(px, py);
	//ctx.lineTo(px, py);
	
	// Sides
	side = 1.5 * Math.hypot(3, 3);
	
	if(angx != 0){
		ang = Math.atan2(angy, angx);
	}
	else{
		ang = Math.PI / 2;
	}
	
	if(angx == 0.0){
		px1 = px - side * Math.cos(ang - 0.5);
		py1 = py - side * Math.sin(ang - 0.5);
		px2 = px - side * Math.cos(ang + 0.5);
		py2 = py - side * Math.sin(ang + 0.5);
	}
	else{
		px1 = px + side * Math.cos(5 * Math.PI / 6 + ang);
		py1 = py - side * Math.sin(5 * Math.PI / 6 + ang);
		px2 = px + side * Math.cos(5 * Math.PI / 6 - ang);
		py2 = py + side * Math.sin(5 * Math.PI / 6 - ang);
	}
	
	ctx.moveTo(px, py);
	ctx.lineTo(px1, py1);
	//ctx.moveTo(px, py);
	ctx.lineTo(px2, py2);
	ctx.lineTo(px, py);
	
	rx = (px + px1 + px2)/3;
	ry = (py + py1 + py2)/3;
	
	ctx.stroke();
	ctx.closePath();
	
	ctx.fillStyle = "#FF0000";
	ctx.fill();
	
	return [rx, ry];
}

function drawTrail(px, py){
	trail.push({x: px, y: py});

	for(i = 0; i < trail.length; i = i + 1){
		drawCircle(trail[i].x, trail[i].y);
	}
}

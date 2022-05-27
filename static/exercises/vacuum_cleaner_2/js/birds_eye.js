// Retreive the canvas elements and context
var mapCanvas = document.getElementById("vacuum-eye"),
	ctx = mapCanvas.getContext("2d");
	
var trail = [],
	coords = [-1, -1];;

var initialPosition;

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
	ctx.arc(x, y, 1.5, 0, 2 * Math.PI);
	ctx.closePath();
	
	ctx.lineWidth = 1.5;
	ctx.strokeStyle = "#0000FF";
	ctx.stroke();
	
	ctx.fillStyle = "#0000FF";
	ctx.fill();
}

// Testing to be carried out with Python interface
function drawTriangle(posx, posy, angx, angy){
	// Store initial position
	if (initialPosition == null) {
		initialPosition = [posx, posy, angx, angy];
	}

	ctx.beginPath();
	
	px = posx;
	py = posy;
	
	// The main line
	ctx.strokeStyle = '#FF0000';
	//ctx.moveTo(px, py);
	//ctx.lineTo(px, py);
	
	// Sides
	side = 1.5 * Math.hypot(2, 2);
	
	if(angx != 0){
		ang = Math.atan2(angy, angx);
	}
	else{
		ang = Math.PI / 2;
	}
	
	px1 = posx + side * Math.cos(2 * Math.PI / 3 + ang);
	py1 = posy - side * Math.sin(2 * Math.PI / 3 + ang);
	px2 = posx + side * Math.cos(2 * Math.PI / 3 - ang);
	py2 = posy + side * Math.sin(2 * Math.PI / 3 - ang);
	px3 = posx + side * Math.cos(ang);
	py3 = posy - side * Math.sin(ang);
	
	ctx.moveTo(px3, py3);
	ctx.lineTo(px1, py1);
	//ctx.moveTo(px, py);
	ctx.lineTo(px2, py2);
	ctx.lineTo(px3, py3);
	
	rx = px;
	ry = py;
	
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

function clearMap(){
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
	trail = [];
}

function restoreInitialPosition() {
	draw(initialPosition[0], initialPosition[1], initialPosition[2], initialPosition[3]);
}

const reset = document.getElementById("reset");
    reset.addEventListener("click", function(){
	clearMap();
});




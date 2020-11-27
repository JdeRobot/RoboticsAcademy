// Retreive the canvas elements and context
var mapCanvas = document.getElementById("birds-eye"),
	ctx = mapCanvas.getContext("2d");
	
var trail = [];

// Complete draw function
function draw(x, y, ax, ay){
	drawTriangle(x, y, ax, ay);
	drawTrail(x, y);
}

// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points
function drawCircle(x, y){
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	
	
	cursor_x = x;
	cursor_y = y;
	
	ctx.beginPath();
	ctx.arc(x, y, 4, 0, 2 * Math.PI);
	ctx.closePath();
	
	ctx.lineWidth = 1.5;
	ctx.strokeStyle = "#0000FF";
	ctx.stroke();
	
	ctx.fillStyle = "#0000FF";
	ctx.fill();
}

// Testing to be carried out with Python interface
function drawTriangle(posx, posy, angx, angy){
	if(posx == 0 && posy == 0){
		return;
	}
	
	ctx.beginPath();
	
	px = posx + angx;
	py = posy + angy;

	// The main line
	ctx.strokeStyle = '#FF0000';
	ctx.moveTo(posx, posy);
	ctx.lineTo(px, py);
	
	// Sides
	side = 3 * Math.hypot(3, 3);
	
	if(posx != 0){
		ang = Math.atan2(angy, angx);
	}
	else{
		ang = Math.PI / 2;
	}
	
	if(angx == 0.0){
		px1 = px + side * Math.cos(ang - 0.5);
		py1 = py + side * Math.sin(ang - 0.5);
		px2 = px + side * Math.cos(ang + 0.5);
		py2 = py + side * Math.sin(ang + 0.5);
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
	
	ctx.stroke();
	
	ctx.closePath();
}

function drawTrail(px, py){
	trail.push((px, py));

	for(i = 0; i < trail.length; i = i + 1){
		drawCircle(px, py);
	}
}

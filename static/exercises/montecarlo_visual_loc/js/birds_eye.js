// Retreive the canvas elements and context
var birdsEye = document.getElementById("birds-eye-montecarlo");
var canvasCtx = birdsEye.getContext("2d");
	
var trail = [],
	coords = [-1, -1];;

// Complete draw function
function draw(x, y, ax, ay){
	canvasCtx.clearRect(0, 0, birdsEye.width, birdsEye.height);	

	drawTrail(coords[0], coords[1]);
	coords = drawTriangle(x, y, ax, ay);
}

// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points
function drawCircle(x, y){
	cursor_x = x;
	cursor_y = y;
	
	canvasCtx.beginPath();
	canvasCtx.arc(x, y, 1.5, 0, 2 * Math.PI);
	canvasCtx.closePath();
	
	canvasCtx.lineWidth = 1.5;
	canvasCtx.strokeStyle = "#0000FF";
	canvasCtx.stroke();
	
	canvasCtx.fillStyle = "#0000FF";
	canvasCtx.fill();
}

// Testing to be carried out with Python interface
function drawTriangle(posx, posy, angx, angy){	
	canvasCtx.beginPath();
	
	px = posx;
	py = posy;
	
	// The main line
	canvasCtx.strokeStyle = '#FF0000';
	//canvasCtx.moveTo(px, py);
	//canvasCtx.lineTo(px, py);
	
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
	
	canvasCtx.moveTo(px3, py3);
	canvasCtx.lineTo(px1, py1);
	//canvasCtx.moveTo(px, py);
	canvasCtx.lineTo(px2, py2);
	canvasCtx.lineTo(px3, py3);
	
	rx = px;
	ry = py;
	
	canvasCtx.stroke();
	canvasCtx.closePath();
	
	canvasCtx.fillStyle = "#FF0000";
	canvasCtx.fill();
	
	return [rx, ry];
}

function drawTrail(px, py){
	trail.push({x: px, y: py});

	for(i = 0; i < trail.length; i = i + 1){
		drawCircle(trail[i].x, trail[i].y);
	}
}

function clearMap(){
	canvasCtx.clearRect(0, 0, birdsEye.width, birdsEye.height);
	trail = [];
}


// Print Particles

function printParticles(particles) {
    var point = [];

    for (var i = 0; i < particles.length; i++) {
		printParticle(particles[i][0], particles[i][1], particles[i][2])
    }
}

function printParticle(mapPositionX, mapPositionY, theta){
	// Draw point
	canvasCtx.beginPath();
	canvasCtx.fillStyle = "blue";
	canvasCtx.strokeStyle = 'blue';
	canvasCtx.arc(mapPositionX, mapPositionY, 2, 0,2*Math.PI);
	canvasCtx.fill();
	canvasCtx.stroke();

	var length = 10;
	var x2 = mapPositionX + Math.cos(Math.PI * -theta / 180) * length;
	var y2 = mapPositionY + Math.sin(Math.PI * -theta / 180) * length;

	canvas_arrow(mapPositionX, mapPositionY, x2, y2);
}

function canvas_arrow(fromx, fromy, tox, toy) {
	canvasCtx.beginPath();
	var headlen = 5; // length of head in pixels
	var dx = tox - fromx;
	var dy = toy - fromy;
	var angle = Math.atan2(dy, dx);
	canvasCtx.moveTo(fromx, fromy);
	canvasCtx.lineTo(tox, toy);
	canvasCtx.lineTo(tox - headlen * Math.cos(angle - Math.PI / 6), toy - headlen * Math.sin(angle - Math.PI / 6));
	canvasCtx.moveTo(tox, toy);
	canvasCtx.lineTo(tox - headlen * Math.cos(angle + Math.PI / 6), toy - headlen * Math.sin(angle + Math.PI / 6));
	canvasCtx.strokeStyle = 'blue';
	canvasCtx.stroke();
}
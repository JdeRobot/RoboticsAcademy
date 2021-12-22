// Retreive the canvas elements and context
var mapCanvas = document.getElementById("globalnav-eye"),
    ctx = mapCanvas.getContext("2d");




// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points


function draw(x, y, ax, ay){

	coords = drawTriangle(x, y, ax, ay);
}


function drawTriangle(posx, posy, angx, angy){
	ctx.clearRect(posx+10, posy-7, -20, 15);
	//ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
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



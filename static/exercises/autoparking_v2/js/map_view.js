// Retreive the canvas elements and context
var mapCanvas = document.getElementById("local-map-lasers"),
    ctx = mapCanvas.getContext("2d");
    
function paintEvent(car, obs, avg, lasers, ranges){
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	
	
	// Draw Car
	drawCar();
	
	// Draw Laser
	drawLaser(lasers[0], ranges[0], "f", "#FF0000");
	drawLaser(lasers[1], ranges[1], "r", "#F7FF00");
	drawLaser(lasers[2], ranges[2], "b", "#0FFF00");
}

function drawCar(){
	carWidth = 40;
	carHeight = 95;
	tiresize = carWidth / 5;
	
	ctx.beginPath();
	
	// Chassis
	ctx.fillStyle = 'black';
	ctx.fillRect(mapCanvas.width/2 - carWidth/2, mapCanvas.height/2 - carHeight/2, carWidth, carHeight);
	
	ctx.stroke();
	
	ctx.closePath();
}

function drawLaser(laser, max_range, pos, color){
	let originx = 0;
	let originy = 0;
	let resizeFactor = 0.8;
	ctx.strokeStyle = color;
	switch(pos) {
		case "f":
			originx = mapCanvas.width/2;
			originy = mapCanvas.height/2 - carHeight/2 + 16;
			break;
		case "r":
			originx = mapCanvas.width/2 + carWidth/2;
			originy = mapCanvas.height/2;
			break;
		case "b":
			originx = mapCanvas.width/2;
			originy = mapCanvas.height/2 + carHeight/2;
			break;
	}

	// Resizes the lasers
	max_range *= resizeFactor;

	for (let d of laser){
		d[0] *= resizeFactor;
		
		if (d[0] > max_range){
			py = originy - max_range * Math.sin(d[1]);
			px = originx + max_range * Math.cos(d[1]);
		}
		else{
			py = originy - d[0] * Math.sin(d[1]);
			px = originx + d[0] * Math.cos(d[1]);
		}
		// Rotates for right and back lasers
		let rotatedPoints;
		switch(pos) {
			case "f":

				break;
			case "r":
				rotatedPoints = rotate(originx, originy, px, py, -90)
				px = rotatedPoints[0];
				py = rotatedPoints[1];
				break;
			case "b":
				rotatedPoints = rotate(originx, originy, px, py, 180)
				px = rotatedPoints[0];
				py = rotatedPoints[1];
				break;
		}

		ctx.beginPath();
		ctx.moveTo(originx, originy);
		ctx.lineTo(px, py);
		ctx.stroke();
		
		ctx.closePath();
	}
}

// Rotates the point
function rotate(cx, cy, x, y, angle) {
    let radians = (Math.PI / 180) * angle;
    let cos = Math.cos(radians);
    let sin = Math.sin(radians);
    let nx = (cos * (x - cx)) + (sin * (y - cy)) + cx;
    let ny = (cos * (y - cy)) - (sin * (x - cx)) + cy;
    return [nx, ny];
}
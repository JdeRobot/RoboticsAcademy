// Retreive the canvas elements and context
var mapCanvas = document.getElementById("local-map"),
    ctx = mapCanvas.getContext("2d");
    
function paintEvent(car, obs, avg, lasers, ranges){
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	
	
	// Draw Car
	drawCar();
	
	// Draw Laser
	drawLaser(lasers[0], ranges[0], "#FF0000");
	drawLaser(lasers[1], ranges[1], "#F7FF00");
	drawLaser(lasers[2], ranges[2], "#0FFF00");
	
	// Draw arrows
	drawArrow(-car[1] , car[0] , "#7CFC00");
	drawArrow(-obs[1] , obs[0] , "#DC143C");
	drawArrow(-avg[1] , avg[0], "#000000");
}

function drawCar(){
	carsize = 60;
	tiresize = carsize / 5;
	
	ctx.beginPath();
	
	// Connector
	ctx.fillStyle = 'black';
	ctx.fillRect(135, 125, 24, 1);
	
	// Chassis
	ctx.fillStyle = 'red';
	ctx.fillRect(143, 120, 8, 12);
	ctx.fillRect(140, 132, 15, 15);
	
	// Tires
	ctx.fillStyle = 'black';
	ctx.fillRect(134, 121, 5, 8);
	ctx.fillRect(155, 121, 5, 8);
	ctx.fillRect(134, 139, 5, 8);
	ctx.fillRect(155, 139, 5, 8);
	
	ctx.stroke();
	
	ctx.closePath();
}

function drawLaser(laser, max_range, color){
	for (let d of laser){
		if (d[0] > max_range){
			py = 120 - max_range * Math.sin(d[1]);
			px = 146.5 + max_range * Math.cos(d[1]);
		}
		else{
			py = 120 - d[0] * Math.sin(d[1]);
			px = 146.5 + d[0] * Math.cos(d[1]);
		}
		
		ctx.beginPath();
		
		ctx.strokeStyle = color;
		ctx.moveTo(146.5, 120);
		ctx.lineTo(px, py);
		ctx.stroke();
		
		ctx.closePath();
	}
}

// Testing to be carried out with Python interface
function drawArrow(posx, posy, color){
	if(posx == 0 && posy == 0){
		return;
	}
	
	px = 146.5 + posx * 20;
	py = 120 - posy * 20;
	
	ctx.beginPath();
	
	// The main line
	ctx.strokeStyle = color;
	ctx.moveTo(146.5, 120);
	ctx.lineTo(px, py);
	
	// Sides
	side = 3 * Math.hypot(posx, posy);
	
	if(posx != 0){
		ang = Math.atan2(posy, posx);
	}
	else{
		ang = Math.PI / 2;
	}
	
	if(posx == 0.0){
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

function drawTarget(posx, posy){
	if(posx == 0 && posy == 0){
		return;
	}
	
	ctx.beginPath();
	
	ctx.strokeStyle = 'yellow';
	ctx.lineWidth = 2;
	
	sx = posx - 7;
	sy = posy - 5;
	ex = posx + 7;
	ey = posy + 5;
	
	ctx.moveTo(sx, sy);
	ctx.lineTo(ex, ey);
	
	sx = posx + 7;
	sy = posy - 5;
	ex = posx - 7;
	ey = posy + 5;
	
	ctx.moveTo(sx, sy);
	ctx.lineTo(ex, ey);
	
	ctx.stroke();
	
	ctx.closePath();
}


//drawArrow(-2, 2, "#7CFC00");
//drawCar();
//drawLaser([[130, Math.PI / 2]]);
//drawTarget(120, 90);

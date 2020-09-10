// Retreive the canvas elements and context
var mapCanvas = document.getElementById("local-map"),
    ctx = mapCanvas.getContext("2d");
    
function paintEvent(target, car, obs, avg, laser, max_range){
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	
	
	// Draw Car
	drawCar();
	
	// Draw Laser
	drawLaser(laser, max_range);
	
	// Draw target
	drawTarget(target[0], target[1]);
	
	// Draw arrows
	drawArrow(car[0], car[1], "#32CD32");
	drawArrow(obs[0], obs[1], "#DC143C");
	drawArrow(avg[0], avg[1], "#000000");
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
}

function drawLaser(laser){
	max_range = 110;
	
	for (let d of laser){
		if (d[0] > max_range){
			py = 120 - max_range * Math.sin(d[1]);
			px = 146.5 + max_range * Math.cos(d[1]);
		}
		else{
			py = 120 - d[0] * Math.sin(d[1]);
			px = 146.5 + d[0] * Math.cos(d[1]);
		}
		
		ctx.strokeStyle = "#6897BB";
		ctx.moveTo(146.5, 120);
		ctx.lineTo(px, py);
		ctx.stroke();
		
		console.log(py);
	}
}

// Testing to be carried out with Python interface
function drawArrow(posx, posy, color){
	if(posx == 0 && posy == 0){
		return;
	}
	
	px = 146.5 + posx * 2;
	py = 120 - posy * 2;
	
	// The main line
	ctx.strokeStyle = color;
	ctx.moveTo(146.5, 120);
	ctx.lineTo(px, py);
	
	// Sides
	sidex = Math.hypot(px, py) / 5;
	sidey = Math.hypot(px, py) / 5;
	
	if(px != 0){
		ang = Math.atan2(py, px);
	}
	else{
		ang = Math.PI / 2;
	}
	
	if(posx > 0.0){
		px1 = px + sidex * Math.cos(Math.PI + ang - 0.5);
		py1 = py + sidey * Math.sin(Math.PI + ang - 0.5);
		px2 = px + sidex * Math.cos(Math.PI + ang + 0.5);
		py2 = py + sidey * Math.sin(Math.PI + ang + 0.5);
	}
	else if(posx == 0){
		px1 = px + sidex * Math.cos(ang - 0.5);
		py1 = py + sidey * Math.sin(ang - 0.5);
		px2 = px + sidex * Math.cos(ang + 0.5);
		py2 = py + sidey * Math.sin(ang + 0.5);
	}
	else{
		px1 = px - sidex * math.cos(ang - 0.5);
		py1 = py - sidey * math.sin(ang - 0.5);
		px2 = px - sidex * math.cos(ang + 0.5);
		py2 = py - sidey * math.sin(ang + 0.5);
	}
	
	ctx.moveTo(px, py);
	ctx.lineTo(px1, py1);
	ctx.lineTo(px2, py2);
	
	ctx.stroke();
	
}

function drawTarget(posx, posy){
	if(posx == 0 && posy == 0){
		return;
	}
	
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
}



//drawCar();
//drawLaser([[130, Math.PI / 2]]);
//drawTarget(120, 90);

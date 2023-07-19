// Retreive the canvas elements and context

let mapCanvas, ctx, f1_center;

export function paintEvent(target, car, obs, avg, laser, max_range){
	mapCanvas = document.getElementById("local-map");
	ctx = mapCanvas.getContext("2d");

	f1_center = [mapCanvas.width/2,  mapCanvas.height/2]
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	
	
	// Draw Car
	drawCar();
	
	// Draw Laser
	drawLaser(laser, max_range);
	
	// Draw target
	drawTarget(target[0], target[1]);
	
	//Draw Distance lines
	drawBorders();
	
	// Draw arrows
	drawArrow(car[0], car[1], "#7CFC00");
	drawArrow(obs[0], obs[1], "#DC143C");
	drawArrow(avg[0], avg[1], "#000000");
}

function drawBorders() { 
	mapCanvas = document.getElementById("local-map");
	ctx = mapCanvas.getContext("2d");

    // draw the stroke 
    let d10 = 10;
    let d25 = 25;
    let scale =5;
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#000000';
    ctx.stroke();
    //Rectangle for 10m
    ctx.strokeRect(f1_center[0]-(d10*scale/2), f1_center[1]-5,d10*scale,d10*scale)
    //Rectangle for 25m
    ctx.strokeRect(f1_center[0]-(d25*scale/2), f1_center[1]-d25,d25*scale,(d25+3)*scale/2)
    
    //Text 10m & 25m local-map canvas
    ctx.font = "14px Comic Sans MS";
    ctx.fillStyle = "#AAAAAA";
    ctx.textAlign = "center";
    let s = d10 + " m";
    ctx.fillText(s, mapCanvas.width/2+50, f1_center[1]-d10); 
    s = d25 + " m";
    ctx.fillText(s, mapCanvas.width/2+50, f1_center[1]-d25); 
}

function drawCar(){
	mapCanvas = document.getElementById("local-map");
	ctx = mapCanvas.getContext("2d");
	let carsize = 60;
	let tiresize = carsize / 5;
	
	ctx.beginPath();
	
	// Calcular la posición de y ajustada del coche
	let d25 = 25;
	let scale =5;
	let border_bottom_y = f1_center[1] - d25 + (d25 + 3) * scale / 2;
	let car_top_y = 120;
	let car_adjusted_y = border_bottom_y - car_top_y;

	// Calcular la posición x ajustada del coche
	let car_width = 24;  // Ancho del coche basado en la longitud del conector
	let car_adjusted_x = f1_center[0] - car_width / 2;
	
	// Connector
	ctx.fillStyle = 'black';
	ctx.fillRect(car_adjusted_x, 125 + car_adjusted_y, car_width, 1);
	
	// Chassis
	ctx.fillStyle = 'red';
	ctx.fillRect(car_adjusted_x + 8, 120 + car_adjusted_y, 8, 12);
	ctx.fillRect(car_adjusted_x + 5, 132 + car_adjusted_y, 15, 15);
	
	// Tires
	ctx.fillStyle = 'black';
	ctx.fillRect(car_adjusted_x - 1, 121 + car_adjusted_y, 5, 8);
	ctx.fillRect(car_adjusted_x + car_width - 4, 121 + car_adjusted_y, 5, 8);
	ctx.fillRect(car_adjusted_x - 1, 139 + car_adjusted_y, 5, 8);
	ctx.fillRect(car_adjusted_x + car_width - 4, 139 + car_adjusted_y, 5, 8);
	
	ctx.stroke();
	
	ctx.closePath();
}


function drawLaser(laser, max_range){
	mapCanvas = document.getElementById("local-map");
	ctx = mapCanvas.getContext("2d");

	// Calcular la posición de y ajustada del coche
	let d25 = 25;
	let scale = 5;
	let border_bottom_y = f1_center[1] - d25 + (d25 + 3) * scale / 2;
	let car_top_y = 120;
	let car_adjusted_y = border_bottom_y - car_top_y;

	// Calcular la posición x ajustada del coche
	let car_width = 24;  // Ancho del coche basado en la longitud del conector
	let car_adjusted_x = f1_center[0] - car_width / 2;

	for (let d of laser){
		let py, px;
		if (d[0] > max_range){
			py = car_top_y + car_adjusted_y - max_range * Math.sin(d[1]);
			px = car_adjusted_x + car_width/2 + max_range * Math.cos(d[1]);
		}
		else{
			py = car_top_y + car_adjusted_y - d[0] * Math.sin(d[1]);
			px = car_adjusted_x + car_width/2 + d[0] * Math.cos(d[1]);
		}
		
		ctx.beginPath();
		
		ctx.strokeStyle = "#6897BB";
		ctx.moveTo(car_adjusted_x + car_width/2, car_top_y + car_adjusted_y);
		ctx.lineTo(px, py);
		ctx.stroke();
		
		ctx.closePath();
	}
}


// Testing to be carried out with Python interface
function drawArrow(posx, posy, color){
	mapCanvas = document.getElementById("local-map");
	ctx = mapCanvas.getContext("2d");

	// Calcular la posición de y ajustada del coche
	let d25 = 25;
	let scale =5;
	let border_bottom_y = f1_center[1] - d25 + (d25 + 3) * scale / 2;
	let car_top_y = 120;
	let car_adjusted_y = border_bottom_y - car_top_y;

	// Calcular la posición x ajustada del coche
	let car_width = 24;  // Ancho del coche basado en la longitud del conector
	let car_adjusted_x = f1_center[0] - car_width / 2;

	if(posx == 0 && posy == 0){
		return;
	}

	let px = car_adjusted_x + car_width / 2 - posy * 20;
	let py = car_top_y + car_adjusted_y - posx * 20;
	
	ctx.beginPath();
	
	// The main line
	ctx.strokeStyle = color;
	ctx.moveTo(car_adjusted_x + car_width / 2, car_top_y + car_adjusted_y);
	ctx.lineTo(px, py);
	
	// Sides
	let side = 3 * Math.hypot(posx, posy);
	
	let ang;
	if(posx != 0){
		ang = Math.atan2(posy, posx);
	}
	else{
		ang = Math.PI / 2;
	}
	
	let px1, py1, px2, py2;
	if(posx == 0.0){
		if(posy <=0){
			px1 = px - side * Math.cos(ang - 0.5);
			py1 = py + side * Math.sin(ang + 0.5);
			px2 = px - side * Math.cos(ang - 0.5);
			py2 = py - side * Math.sin(ang + 0.5);
		}
		else{
			px1 = px + side * Math.cos(ang - 0.5);
			py1 = py + side * Math.sin(ang + 0.5);
			px2 = px + side * Math.cos(ang - 0.5);
			py2 = py - side * Math.sin(ang + 0.5);
		}
	}
	else{
		px1 = px - side * Math.cos(5 * Math.PI / 6 + ang);
		py1 = py + side * Math.sin(5 * Math.PI / 6 + ang);
		px2 = px + side * Math.cos(5 * Math.PI / 6 - ang);
		py2 = py + side * Math.sin(5 * Math.PI / 6 - ang);
	}
	
	ctx.moveTo(px, py);
	ctx.lineTo(px1, py1);
	ctx.lineTo(px2, py2);
	ctx.lineTo(px, py);
	
	ctx.stroke();
	
	ctx.closePath();
}



function drawTarget(posx, posy){
	mapCanvas = document.getElementById("local-map");
	ctx = mapCanvas.getContext("2d");
	if(posx == 0 && posy == 0){
		return;
	}
	console.log(posx, posy)
	ctx.beginPath();
	
	ctx.strokeStyle = 'yellow';
	ctx.lineWidth = 2;
	
	let sx = posx - 7;
	let sy = posy - 5;
	let ex = posx + 7;
	let ey = posy + 5;
	
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

// Retreive the canvas elements and context
var mapCanvas = document.getElementById("local-map"),
    ctx = mapCanvas.getContext("2d");
    
function paintEvent(target, car, obs, avg){
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	
	
	// Draw Car
	drawCar();
	
	// Draw Laser
	drawLaser();
	
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

function 

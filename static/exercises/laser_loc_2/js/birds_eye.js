// Retreive the canvas elements and context
var mapCanvas = document.getElementById("birds-eye-mapping"),
	ctx = mapCanvas.getContext("2d");
	
	
var trail = [],
	coords = [-1, -1];
var resolution = 1;
// Complete draw function
function draw(pos, contorno, laser_data, sonar_sensor_point, pos_vertices, laser_global,
	approximated_robot_pose, particles, estimated_laser){
	mapCanvas.width = 769;
	mapCanvas.height = 729;
	
	// clearMap();
	// drawTrail(coords[0], coords[1]);
	drawLaser(laser_data, laser_global);
	drawAmigobot(pos, contorno);
	drawSonar(sonar_sensor_point, pos_vertices);
	drawAMCL(approximated_robot_pose, particles);
	// drawEstimatedLaser(estimated_laser);
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
	/*Crea un nuevo trazo. Una vez creado, los comandos
	de dibujo futuros son aplicados dentro del trazo y 
	usados para construir el nuevo trazo hacia arriba*/
	var ang = 0;
	ctx.beginPath();
	
	px = posx;
	py = posy;
	
	// The main line
	ctx.strokeStyle = '#FF0000';
	
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

function drawAmigobot(pos, contorno){
	ctx.beginPath();
	ctx.strokeStyle = "#000000";
	// First: draw the circle
	ctx.arc(pos[0], pos[1], 5, 0, 2 * Math.PI);
	ctx.stroke();
	ctx.fillStyle = "#000000";
	ctx.fill();
	ctx.closePath();
	   
	// Second: draw the triangle
	ctx.beginPath();
	ctx.strokeStyle = "#FFFB00";
	ctx.moveTo(contorno[0][0], contorno[0][1]);
	ctx.lineTo(contorno[1][0], contorno[1][1]);
	ctx.lineTo(contorno[2][0], contorno[2][1]);
	ctx.lineTo(contorno[0][0], contorno[0][1]);
	ctx.stroke();
	ctx.fillStyle = "#FFFB00";
	ctx.fill();
	ctx.closePath();
}
function drawLaser(dataLaser, laser_global){
	for(let d of dataLaser){
		ctx.beginPath();
		ctx.strokeStyle = "#FF2D00";
		ctx.moveTo(laser_global[0], laser_global[1]);
		ctx.lineTo(d[0], d[1]);
		ctx.stroke();
		ctx.closePath();
	}
}
function drawSonar(sonar_sensor_point, pos_vertices){
	let px_sensor = [];
	let py_sensor = [];
	let j = 0;
	let z = 0;
	// alert(`[close] Connection closed cleanly`);
	for(let k of sonar_sensor_point){
		px_sensor[j] = k[0];
		py_sensor[j] = k[1];
		j++;	
	}
	px_v = pos_vertices[0]
	for(let d of pos_vertices){

		ctx.beginPath();
		ctx.strokeStyle = "#4AA434";
		ctx.fillStyle = "#4AA434";
		ctx.moveTo(px_sensor[z], py_sensor[z]);
		ctx.lineTo(d[0],d[1]);
		ctx.lineTo(d[2],d[3]);
		ctx.lineTo(px_sensor[z], py_sensor[z]);
		ctx.stroke();
		ctx.fill();
		ctx.closePath();
		z++;
	}
}
function drawAMCL(approximated_robot_pose, particles){
	// alert(`[close] Connection closed cleanly`);
	for(let d of particles){
		ctx.beginPath();
		ctx.strokeStyle = "#000000";
		if(d[4]<=0.02){
			ctx.fillStyle = "#00FF2E";
		}
		else if(d[4]>0.02 && d[4]<=0.04){
			ctx.fillStyle = "#00B821";
		}
		else if(d[4]>0.04 && d[4]<=0.06){
			ctx.fillStyle = "#008C19";
		}
		else if(d[4]>0.06 && d[4]<=0.08){
			ctx.fillStyle = "#005910";
		}
		else if(d[4]>0.08 && d[4]<=0.1){
			ctx.fillStyle = "#002306";
		}
		ctx.arc(d[0] / resolution , 729 - d[1] / resolution, 3, 0, 2 * Math.PI);

		xr = Math.cos(0)*15;
		yr = Math.sin(0)*15;
		
		px = d[0] + Math.cos(d[2]) * xr - Math.sin(d[2]) * yr;
		py = d[1] + Math.sin(d[2]) * xr + Math.cos(d[2]) * yr;
		
		ctx.moveTo(d[0] / resolution, 729 - (d[1] / resolution));
		ctx.lineTo(px, 729 - py);
		ctx.stroke();
		ctx.fill();
		ctx.closePath();
	}
	ctx.beginPath();
	ctx.strokeStyle = "#000000";
	ctx.fillStyle = "#0000FF";
	ctx.arc(approximated_robot_pose[0] / resolution, 729 - approximated_robot_pose[1] / resolution, 8, 0, 2 * Math.PI);
	
	xr = Math.cos(0)*20;
	yr = Math.sin(0)*20;
	
	px = approximated_robot_pose[0] + Math.cos(approximated_robot_pose[2]) * xr - Math.sin(approximated_robot_pose[2]) * yr;
	py = approximated_robot_pose[1] + Math.sin(approximated_robot_pose[2]) * xr + Math.cos(approximated_robot_pose[2]) * yr;

	ctx.moveTo(approximated_robot_pose[0] / resolution, 729 - approximated_robot_pose[1] / resolution)
	ctx.lineTo(px, 729 - py)
	ctx.stroke();
	ctx.fill();
	ctx.closePath();
}
// function drawEstimatedLaser(estimated_laser){
// 	var j = 0;
// 	for(let d of estimated_laser[0]){
// 		for(let i = 0; i < 5; i++){
// 			ctx.beginPath();
// 			ctx.strokeStyle = "#002AFE";
// 			ctx.moveTo(d[0], 729 - d[1]);
// 			ctx.lineTo(estimated_laser[1][j][i][0], 729 - estimated_laser[1][j][i][1]);
// 			ctx.stroke();
// 			ctx.closePath();
// 		}
// 		j = j + 1;
// 	}
// }
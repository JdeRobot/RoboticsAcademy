// Retrieve the canvas elements and context


export function draw(x, y, ax, ay){
 
	let coords = drawTriangle(x, y, ax, ay); // declare variable 'coords'
}

function drawTriangle(posx, posy, angx, angy){
  var mapCanvas = document.getElementById("globalnav-eye"),
  ctx = mapCanvas.getContext("2d");
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
	ctx.beginPath();

	let px = posx;
	let py = posy;

	// The main line
	ctx.strokeStyle = '#FF0000';
	//ctx.moveTo(px, py);
	//ctx.lineTo(px, py);

	// Sides
	let side = 1.5 * Math.hypot(2, 2);
	let ang;

	if(angx != 0){
		ang = Math.atan2(angy, angx);
	}
	else{
		ang = Math.PI / 2;
	}

	let px1 = posx + side * Math.cos(2 * Math.PI / 3 + ang);
	let py1 = posy - side * Math.sin(2 * Math.PI / 3 + ang);
	let px2 = posx + side * Math.cos(2 * Math.PI / 3 - ang);
	let py2 = posy + side * Math.sin(2 * Math.PI / 3 - ang);
	let px3 = posx + side * Math.cos(ang);
	let py3 = posy - side * Math.sin(ang);

	ctx.moveTo(px3, py3);
	ctx.lineTo(px1, py1);
	//ctx.moveTo(px, py);
	ctx.lineTo(px2, py2);
	ctx.lineTo(px3, py3);

	let rx = px;
	let ry = py;

	ctx.stroke();
	ctx.closePath();

	ctx.fillStyle = "#FF0000";
	ctx.fill();

	return [rx, ry];
}

export function generatePath(data){
	var mapCanvas = document.getElementById("globalnav-eye"),
	ctx = mapCanvas.getContext("2d");
	clearPath();
	drawTargetPosition();
	data = data;
	if (data == null){
	   return null
	}
	let minx,miny,maxx,maxy;
	miny = minx = Infinity
	maxx = maxy = -Infinity;
	data.forEach(point => {
		  minx = Math.min(minx,point[0]);
		  miny = Math.min(miny,point[1]);
		  maxx = Math.max(maxx,point[0]);
		  maxy = Math.max(maxy,point[1]);
	   });
	let rangeX = maxx - minx;
	let rangeY = maxy - miny;
	let range = Math.max(rangeX,rangeY);
	let scale = Math.min(mapCanvas.width,mapCanvas.height);

	for (let i = 0; i < data.length-1; i++) {
	   ctx.beginPath();
	   ctx.moveTo(data[i][0], data[i][1]);
	   ctx.strokeStyle = "#008000";
	   ctx.strokeWidth = 100;
		  let x = data[i+1][0];
		  let y = data[i+1][1];
		  //x = ((x-minx) / range) * scale;
		  //y = ((y-miny) / range) * scale;
		  ctx.lineTo(x,y);
		  ctx.stroke();
	   }
}

function clearPath() {
	var mapCanvas = document.getElementById("globalnav-eye"),
	ctx = mapCanvas.getContext("2d");
 	ctx.clearRect(0,0,mapCanvas.width,mapCanvas.height);
}

function drawTargetPosition() {
	var mapCanvas = document.getElementById("globalnav-eye"),
	ctx = mapCanvas.getContext("2d");
 if (cursorXMap != 0 || cursorYMap != 0) {
	ctx.beginPath();
	ctx.strokeStyle = "#0000FF";
 
	ctx.moveTo((cursorXMap - 8), (cursorYMap - 8));
	ctx.lineTo((cursorXMap + 8), (cursorYMap + 8));
 
	ctx.moveTo((cursorXMap + 8), (cursorYMap - 8));
	ctx.lineTo((cursorXMap - 8), (cursorYMap + 8));
	ctx.stroke();
	ctx.closePath();
 }
}
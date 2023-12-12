
var trail = [],
	coords = [-1, -1];;

var initialPosition;

// Complete draw function
export function draw(mapCanvas, x, y, ax, ay, xUser, yUser, axUser, ayUser){
	let ctx = mapCanvas.getContext("2d")
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
	drawTriangle(ctx, x, y, ax, ay, 2);
    if(xUser > 0)
    drawTriangle(ctx, xUser, yUser, axUser, ayUser, 1)
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
	ctx.strokeStyle = "#ff8c00";
	ctx.stroke();
	
	ctx.fillStyle = "#ff8c00";
	ctx.fill();
}

function drawTriangle(ctx, posx, posy, angx, angy, type) {
    if (initialPosition == null) {
        initialPosition = [posx, posy, angx, angy];
    }

    ctx.beginPath();

    let px = posx;
    let py = posy;

    // Triangle type
    let side;
    if (type == 1) {
        ctx.strokeStyle = "#0010bf";    
        side = 1.32 * Math.hypot(2, 2);
    }
    else if (type == 2) {
        ctx.strokeStyle = "#FF0000";    
        side = 1.67 * Math.hypot(2, 2);
    }

    let ang;
    if (angx != 0) {
        ang = Math.atan2(angy, angx);
    } else {
        ang = Math.PI / 2;
    }
    ang += 0.25;

    let px1 = posx + side * Math.cos(2 * Math.PI / 3 + ang);
    let py1 = posy - side * Math.sin(2 * Math.PI / 3 + ang);
    let px2 = posx + side * Math.cos(2 * Math.PI / 3 - ang);
    let py2 = posy + side * Math.sin(2 * Math.PI / 3 - ang);
    let px3 = posx + side * Math.cos(ang);
    let py3 = posy - side * Math.sin(ang);

    ctx.moveTo(px3, py3);
    ctx.lineTo(px1, py1);
    ctx.lineTo(px2, py2);
    ctx.lineTo(px3, py3);

    ctx.stroke();
    ctx.closePath();

    if (type == 1) {
        ctx.fillStyle = "#0010bf";
        ctx.fill();
    }

    return [px, py];
}


function drawTrail(px, py){
	trail.push({x: px, y: py});

	for(i = 0; i < trail.length; i = i + 1){
		drawCircle(trail[i].x, trail[i].y);
	}
}

export function clearMap(mapCanvas){
    let ctx = mapCanvas.getContext("2d")
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
	trail = [];
}

function restoreInitialPosition() {
	draw(initialPosition[0], initialPosition[1], initialPosition[2], initialPosition[3]);
}




// Print Particles

export function printParticles(mapCanvas, particles) {
    var point = [];

    for (var i = 0; i < particles.length; i++) {
        printParticle(mapCanvas, particles[i][0], particles[i][1], particles[i][2])
    }
}

function printParticle(mapCanvas, mapPositionX, mapPositionY, theta){
	let ctx = mapCanvas.getContext("2d")
    ctx.beginPath();
    ctx.fillStyle = "#4553e8";
    ctx.strokeStyle = '#4553e8';
    ctx.arc(mapPositionX, mapPositionY, 1, 0,2*Math.PI);
    ctx.fill();
    ctx.stroke();

    var length = 5;
    var x2 = mapPositionX + Math.cos(Math.PI * -theta / 180) * length;
    var y2 = mapPositionY + Math.sin(Math.PI * -theta / 180) * length;

    canvas_arrow(mapCanvas, mapPositionX, mapPositionY, x2, y2);
	ctx.closePath();
}

function canvas_arrow(mapCanvas, fromx, fromy, tox, toy) {
	let ctx = mapCanvas.getContext("2d")
    ctx.beginPath();
    var headlen = 5; // length of head in pixels
    var dx = tox - fromx;
    var dy = toy - fromy;
    var angle = Math.atan2(dy, dx);
    ctx.moveTo(fromx, fromy);
    ctx.lineTo(tox, toy);
//  canvasCtx.lineTo(tox - headlen * Math.cos(angle - Math.PI / 6), toy - headlen * Math.sin(angle - Math.PI / 6));
//  canvasCtx.moveTo(tox, toy);
//  canvasCtx.lineTo(tox - headlen * Math.cos(angle + Math.PI / 6), toy - headlen * Math.sin(angle + Math.PI / 6));
	ctx.strokeStyle = '#4553e8';
	ctx.stroke();
}

// Retreive the canvas elements and context
var mapCanvas = document.getElementById("birds-eye"),
    ctx = mapCanvas.getContext("2d");

var initialPosition;

// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points
function drawCircle(x, y){
	if (initialPosition == null) {
		inialPosition = [x, y];
	}
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	
	
	cursor_x = x;
	cursor_y = y;
	
	ctx.beginPath();
	ctx.arc(x, y, 2, 0, 2 * Math.PI);
	ctx.closePath();
	
	ctx.lineWidth = 1.5;
	ctx.strokeStyle = "#666666";
	ctx.stroke();
	
	ctx.fillStyle = "#FF0000";
	ctx.fill();
}

// Reset to the first position received
function drawInitialPosition() {
	drawCircle(initialPosition[0], initialPosition[1]);
}
// Retreive the canvas elements and context
var mapCanvas = document.getElementById("birds-eye"),
    ctx = mapCanvas.getContext("2d");

// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points
function drawCircle(x, y){
	ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);	
	
	cursor_x = x;
	cursor_y = y;
	
	ctx.beginPath();
	ctx.arc(x, y, 4, 0, 2 * Math.PI);
	ctx.closePath();
	
	ctx.lineWidth = 1.5;
	ctx.strokeStyle = "#0000FF";
	ctx.stroke();
	
	ctx.fillStyle = "#0000FF";
	ctx.fill();
}

function drawTriangle()

// drawCircle(12, 23);

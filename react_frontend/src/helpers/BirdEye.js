// Retreive the canvas elements and context
let mapCanvas;
let ctx;
let trail = [],
  coords = [-1, -1];
let initialPosition;

// Complete draw function

export function draw(canvas, x, y, ax, ay) {
  mapCanvas = canvas;
  ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

  drawTrail(coords[0], coords[1]);
  coords = drawTriangle(x, y, ax, ay);
}

// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points
function drawCircle(x, y) {
  // const cursor_x = x;
  // const cursor_y = y;

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
function drawTriangle(posx, posy, angx, angy) {
  // Store initial position
  if (initialPosition == null) {
    initialPosition = [posx, posy, angx, angy];
  }

  ctx.beginPath();

  let px = posx;
  let py = posy;

  // The main line
  ctx.strokeStyle = "#FF0000";
  //ctx.moveTo(px, py);
  //ctx.lineTo(px, py);

  // Sides
  const side = 1.5 * Math.hypot(2, 2);
  let ang;
  if (angx !== 0) {
    ang = Math.atan2(angy, angx);
  } else {
    ang = Math.PI / 2;
  }

  let px1 = posx + side * Math.cos((2 * Math.PI) / 3 + ang);
  let py1 = posy - side * Math.sin((2 * Math.PI) / 3 + ang);
  let px2 = posx + side * Math.cos((2 * Math.PI) / 3 - ang);
  let py2 = posy + side * Math.sin((2 * Math.PI) / 3 - ang);
  let px3 = posx + side * Math.cos(ang);
  let py3 = posy - side * Math.sin(ang);

  ctx.moveTo(px3, py3);
  ctx.lineTo(px1, py1);
  //ctx.moveTo(px, py);
  ctx.lineTo(px2, py2);
  ctx.lineTo(px3, py3);

  const rx = px;
  const ry = py;

  ctx.stroke();
  ctx.closePath();

  ctx.fillStyle = "#FF0000";
  ctx.fill();

  return [rx, ry];
}

function drawTrail(px, py) {
  trail.push({ x: px, y: py });

  for (var i = 0; i < trail.length; i = i + 1) {
    drawCircle(trail[i].x, trail[i].y);
  }
}

export function clearMap() {
  ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  trail = [];
}

export function restoreInitialPosition() {
  draw(
    initialPosition[0],
    initialPosition[1],
    initialPosition[2],
    initialPosition[3]
  );
}

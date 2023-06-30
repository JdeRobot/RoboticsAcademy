// Retreive the canvas elements and context
let mapCanvas;
let ctx;
let initialPosition;

// Complete draw function

export function draw(canvas, x, y, ax, ay) {
  mapCanvas = canvas;
  ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  coords = drawTriangle(x, y, ax, ay);
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


export function restoreInitialPosition() {
  draw(
    initialPosition[0],
    initialPosition[1],
    initialPosition[2],
    initialPosition[3]
  );
}

// Retreive the canvas elements and context
let mapCanvas = null;
let ctx = null;

// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points

export function draw(map, x, y, ax, ay) {
  mapCanvas = map;
  ctx = mapCanvas.getContext("2d");
  ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height); 
  console.log(x,y,ax,ay)
  drawTriangle(x, y, ax, ay);
}

function drawTriangle(posx, posy, angx, angy) {
  console.log(ctx)
  ctx.clearRect(posx + 10, posy - 7, -20, 15);
  //ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  ctx.beginPath();

  let px = posx;
  let py = posy;

  // The main line
  ctx.strokeStyle = "#FF0000";
  //ctx.moveTo(px, py);
  //ctx.lineTo(px, py);

  // Sides
  let side = 1.5 * Math.hypot(2, 2);
  let ang = null;

  if (angx != 0) {
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

  let rx = px;
  let ry = py;

  ctx.stroke();
  ctx.closePath();

  ctx.fillStyle = "#FF0000";
  ctx.fill();

  return [rx, ry];
}

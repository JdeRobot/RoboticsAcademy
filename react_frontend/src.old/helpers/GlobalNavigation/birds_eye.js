// Retrieve the canvas elements and context

let ctx;
export function draw(mapCanvas, x, y, ax, ay) {
  ctx = mapCanvas.getContext("2d");
  return drawTriangle(x, y, ax, ay);
}

// Function to draw triangle
// Given the coordinates of center, and
// the angle towards which it points

function drawTriangle(posX, posY, angX, angY) {
  ctx.clearRect(posX + 10, posY - 7, -20, 15);
  //ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  ctx.beginPath();

  let px = posX;
  let py = posY;

  // The main line
  ctx.strokeStyle = "#FF0000";
  //ctx.moveTo(px, py);
  //ctx.lineTo(px, py);

  // Sides
  const side = 1.5 * Math.hypot(2, 2);
  let ang;
  if (angX !== 0) {
    ang = Math.atan2(angY, angX);
  } else {
    ang = Math.PI / 2;
  }

  const px1 = posX + side * Math.cos((2 * Math.PI) / 3 + ang);
  const py1 = posY - side * Math.sin((2 * Math.PI) / 3 + ang);
  const px2 = posX + side * Math.cos((2 * Math.PI) / 3 - ang);
  const py2 = posY + side * Math.sin((2 * Math.PI) / 3 - ang);
  const px3 = posX + side * Math.cos(ang);
  const py3 = posY - side * Math.sin(ang);

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

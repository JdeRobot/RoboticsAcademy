let ctx, width, height, carWidth, carHeight;

export function configMap(mapCanvas, car, obs, avg, lasers, ranges) {
  ctx = mapCanvas.getContext("2d");
  width = mapCanvas.width;
  height = mapCanvas.height;
  paintEvent(car, obs, avg, lasers, ranges);
}

function paintEvent(car, obs, avg, lasers, ranges) {
  ctx.clearRect(0, 0, width, height);

  // Draw Car
  drawCar();

  // Draw Laser
  drawLaser(lasers[0], ranges[0], "f", "#FF0000");
  drawLaser(lasers[1], ranges[1], "r", "#F7FF00");
  drawLaser(lasers[2], ranges[2], "b", "#0FFF00");
}

function drawCar() {
  carWidth = 40;
  carHeight = 95;
  // const tireSize = carWidth / 5;

  ctx.beginPath();

  // Chassis
  ctx.fillStyle = "black";
  ctx.fillRect(
    width / 2 - carWidth / 2,
    height / 2 - carHeight / 2,
    carWidth,
    carHeight
  );

  ctx.stroke();

  ctx.closePath();
}

function drawLaser(laser, max_range, pos, color) {
  let originX = 0;
  let originY = 0;
  let resizeFactor = 0.8;
  ctx.strokeStyle = color;
  switch (pos) {
    case "f":
      originX = width / 2;
      originY = height / 2 - carHeight / 2 + 16;
      break;
    case "r":
      originX = width / 2 + carWidth / 2;
      originY = height / 2;
      break;
    case "b":
      originX = width / 2;
      originY = height / 2 + carHeight / 2;
      break;
  }

  // Resizes the lasers
  max_range *= resizeFactor;

  for (let d of laser) {
    d[0] *= resizeFactor;
    let px, py;
    if (d[0] > max_range) {
      py = originY - max_range * Math.sin(d[1]);
      px = originX + max_range * Math.cos(d[1]);
    } else {
      py = originY - d[0] * Math.sin(d[1]);
      px = originX + d[0] * Math.cos(d[1]);
    }
    // Rotates for right and back lasers
    let rotatedPoints;
    switch (pos) {
      case "f":
        break;
      case "r":
        rotatedPoints = rotate(originX, originY, px, py, -90);
        px = rotatedPoints[0];
        py = rotatedPoints[1];
        break;
      case "b":
        rotatedPoints = rotate(originX, originY, px, py, 180);
        px = rotatedPoints[0];
        py = rotatedPoints[1];
        break;
    }

    ctx.beginPath();
    ctx.moveTo(originX, originY);
    ctx.lineTo(px, py);
    ctx.stroke();

    ctx.closePath();
  }
}

// Rotates the point
function rotate(cx, cy, x, y, angle) {
  let radians = (Math.PI / 180) * angle;
  let cos = Math.cos(radians);
  let sin = Math.sin(radians);
  let nx = cos * (x - cx) + sin * (y - cy) + cx;
  let ny = cos * (y - cy) - sin * (x - cx) + cy;
  return [nx, ny];
}

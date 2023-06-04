let ctx, f1_center, width, height;

export function configMap(mapCanvas, target, car, obs, avg, laser, max_range) {
  ctx = mapCanvas.getContext("2d");
  f1_center = [mapCanvas.width / 2, mapCanvas.height / 2];
  width = mapCanvas.width;
  height = mapCanvas.height;
  paintEvent(target, car, obs, avg, laser, max_range);
}

function paintEvent(target, car, obs, avg, laser, max_range) {
  ctx.clearRect(0, 0, width, height);

  // Draw Car
  drawCar();

  // Draw Laser
  drawLaser(laser, max_range);

  // Draw target
  drawTarget(target[0], target[1]);

  //Draw Distance lines
  drawBorders();

  // Draw arrows
  drawArrow(car[0], car[1], "#7CFC00");
  drawArrow(obs[0], obs[1], "#DC143C");
  drawArrow(avg[0], avg[1], "#000000");
}

function drawBorders() {
  // draw the stroke
  const d10 = 10;
  const d25 = 25;
  const scale = 5;
  ctx.lineWidth = 1;
  ctx.strokeStyle = "#000000";
  ctx.stroke();
  //Rectangle for 10m
  ctx.strokeRect(
    f1_center[0] - (d10 * scale) / 2,
    f1_center[1] - 5,
    d10 * scale,
    d10 * scale
  );
  //Rectangle for 25m
  ctx.strokeRect(
    f1_center[0] - (d25 * scale) / 2,
    f1_center[1] - d25,
    d25 * scale,
    ((d25 + 3) * scale) / 2
  );

  //Text 10m & 25m local-map canvas
  ctx.font = "14px Comic Sans MS";
  ctx.fillStyle = "#AAAAAA";
  ctx.textAlign = "center";
  let s = d10 + " m";
  ctx.fillText(s, width / 2 + 50, f1_center[1] - d10);
  s = d25 + " m";
  ctx.fillText(s, width / 2 + 50, f1_center[1] - d25);
}

function drawCar() {
  // const carSize = 60;
  // const tireSize = carSize / 5;

  ctx.beginPath();

  // Connector
  ctx.fillStyle = "black";
  ctx.fillRect(135, 125, 24, 1);

  // Chassis
  ctx.fillStyle = "red";
  ctx.fillRect(143, 120, 8, 12);
  ctx.fillRect(140, 132, 15, 15);

  // Tires
  ctx.fillStyle = "black";
  ctx.fillRect(134, 121, 5, 8);
  ctx.fillRect(155, 121, 5, 8);
  ctx.fillRect(134, 139, 5, 8);
  ctx.fillRect(155, 139, 5, 8);

  ctx.stroke();

  ctx.closePath();
}

function drawLaser(laser, max_range) {
  let px, py;
  for (let d of laser) {
    if (d[0] > max_range) {
      py = 120 - max_range * Math.sin(d[1]);
      px = 146.5 + max_range * Math.cos(d[1]);
    } else {
      py = 120 - d[0] * Math.sin(d[1]);
      px = 146.5 + d[0] * Math.cos(d[1]);
    }

    ctx.beginPath();

    ctx.strokeStyle = "#6897BB";
    ctx.moveTo(146.5, 120);
    ctx.lineTo(px, py);
    ctx.stroke();

    ctx.closePath();
  }
}

// Testing to be carried out with Python interface
function drawArrow(posX, posY, color) {
  let px, py;
  if (posX === 0 && posY === 0) {
    return;
  }

  px = 146.5 - posY * 20;
  py = 120 - posX * 20;

  ctx.beginPath();

  // The main line
  ctx.strokeStyle = color;
  ctx.moveTo(146.5, 120);
  ctx.lineTo(px, py);

  // Sides
  const side = 3 * Math.hypot(posX, posY);
  let ang, px1, py1, px2, py2;
  if (posX !== 0) {
    ang = Math.atan2(posY, posX);
  } else {
    ang = Math.PI / 2;
  }

  if (posX === 0.0) {
    if (posY <= 0) {
      px1 = px - side * Math.cos(ang - 0.5);
      py1 = py + side * Math.sin(ang + 0.5);
      px2 = px - side * Math.cos(ang - 0.5);
      py2 = py - side * Math.sin(ang + 0.5);
    } else {
      px1 = px + side * Math.cos(ang - 0.5);
      py1 = py + side * Math.sin(ang + 0.5);
      px2 = px + side * Math.cos(ang - 0.5);
      py2 = py - side * Math.sin(ang + 0.5);
    }
  } else {
    px1 = px - side * Math.cos((5 * Math.PI) / 6 + ang);
    py1 = py + side * Math.sin((5 * Math.PI) / 6 + ang);
    px2 = px + side * Math.cos((5 * Math.PI) / 6 - ang);
    py2 = py + side * Math.sin((5 * Math.PI) / 6 - ang);
  }

  ctx.moveTo(px, py);
  ctx.lineTo(px1, py1);
  //ctx.moveTo(px, py);
  ctx.lineTo(px2, py2);
  ctx.lineTo(px, py);

  ctx.stroke();

  ctx.closePath();
}

function drawTarget(posX, posY) {
  if (posX === 0 && posY === 0) {
    return;
  }

  ctx.beginPath();

  ctx.strokeStyle = "yellow";
  ctx.lineWidth = 2;
  let sx, sy, ex, ey;
  sx = posX - 7;
  sy = posY - 5;
  ex = posX + 7;
  ey = posY + 5;

  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);

  sx = posX + 7;
  sy = posY - 5;
  ex = posX - 7;
  ey = posY + 5;

  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);

  ctx.stroke();

  ctx.closePath();
}

//drawArrow(0, 5, "#7CFC00");
//drawCar();
//drawLaser([[130, Math.PI / 2]]);
//drawTarget(120, 90);

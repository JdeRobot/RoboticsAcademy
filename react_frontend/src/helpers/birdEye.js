export const drawCircle = (x, y, initialPosition, mapCanvas) => {
  if (initialPosition == null) {
    initialPosition = [x, y];
  }
  console.log("Place Robot in the starting position");
  let ctx = mapCanvas.getContext("2d");
  ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

  // cursor_x = x;
  // cursor_y = y;

  ctx.beginPath();
  ctx.arc(x, y, 2, 0, 2 * Math.PI);
  ctx.closePath();
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = "#666666";
  ctx.stroke();

  ctx.fillStyle = "#FF0000";
  ctx.fill();
  return initialPosition;
};

export const drawInitialPosition = (initialPosition, mapCanvas) => {
  drawCircle(
    initialPosition[0],
    initialPosition[1],
    initialPosition,
    mapCanvas
  );
};

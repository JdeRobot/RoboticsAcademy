export const drawCircle = (x, y, initialPosition, mapCanvas) => {
  if (initialPosition == null) {
    initialPosition = [x, y];
  }
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

export const scaleToFit = (img, ctx, canvas) => {
  // get the scale
  const scale = Math.min(canvas.width / img.width, canvas.height / img.height);
  // get the top left position of the image
  const x = canvas.width / 2 - (img.width / 2) * scale;
  const y = canvas.height / 2 - (img.height / 2) * scale;
  ctx.drawImage(img, x, y, img.width * scale, img.height * scale);
};

export const drawInitialPosition = (initialPosition, mapCanvas) => {
  drawCircle(
    initialPosition[0],
    initialPosition[1],
    initialPosition,
    mapCanvas
  );
};

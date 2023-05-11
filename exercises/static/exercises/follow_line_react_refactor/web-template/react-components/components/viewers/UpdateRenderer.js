function drawCar(ctx, x, y)
{
  ctx.beginPath();
  ctx.arc(x, y, 3, 0, 2 * Math.PI, false);

  ctx.lineWidth = 1.5;
  ctx.strokeStyle = "#666666";
  ctx.stroke();

  ctx.fillStyle = "#FF0000";
  ctx.fill();
}

function updateRenderer(image, circuitImage, carPosition) {
  const width = 300;
  const height = 150;

  this.canvas.width = width;
  this.canvas.height = height;
  // this.clearRect(0, 0, width, height);
  this.drawImage(image, 0, 0, width, height);
  this.drawImage(circuitImage, 0, 0, width, height);
  drawCar(this, carPosition[0], carPosition[1]);
}

function updateRendererNew(image, circuitImage, carPosition) {
  const width = this.canvas.width;
  const height = this.canvas.height;

  this.clearRect(0, 0, width, height);
  this.drawImage(image, 0, 0, width, height);
  this.drawImage(circuitImage, 16, 16, width, circuitImage.height);
  drawCar(this, carPosition[0], carPosition[1]);
}

export default updateRenderer;
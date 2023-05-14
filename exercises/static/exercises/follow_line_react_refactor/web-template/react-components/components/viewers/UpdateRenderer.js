function UpdateRenderer()
{
  const circuitReferenceWidth = 300;
  const circuitReferenceHeight = 150;
  const circuitScaleX = 0.5;
  const circuitScaleY = 0.5;
  const circuitMarginX = 16;
  const circuitMarginY = 16;

  let currentTick = null;

  this.init = (width, height, canvas, imageRef, circuitImageRef, circuitPositionRef) => {
    this.ctx = canvas.getContext("2d");
    this.canvas = canvas;
    this.imageRef = imageRef;
    this.circuitImageRef = circuitImageRef;
    this.circuitPositionRef = circuitPositionRef;

    this.width = width;
    this.height = height;
    this.circuitWidth = (this.width * circuitScaleX) - (2 * circuitMarginX);
    this.circuitHeight = (this.height * circuitScaleY) - (2 * circuitScaleY);

    this.scaleX = this.circuitWidth / circuitReferenceWidth;
    this.scaleY = this.circuitHeight / circuitReferenceHeight;
  }

  this.run = () => {
    currentTick = requestAnimationFrame(this.tick);
  }

  this.stop = () => {
    if(currentTick != null) {
      cancelAnimationFrame(currentTick);
    }
  }

  this.tick = () => {
    // clear canvas
    this.ctx.canvas.width = this.width;
    this.ctx.canvas.height = this.height;

    // Draw current image
    this.drawCameraImage();

    // Draw circuit image and car position point
    this.drawAerialView();

    currentTick = requestAnimationFrame(this.tick);
  };

  this.drawCameraImage = () => {
    this.ctx.drawImage(this.imageRef.current, 0, 0, this.width, this.height);
  };

  this.drawAerialView = () => {
    this.ctx.drawImage(this.circuitImageRef.current, circuitMarginX, circuitMarginY, this.circuitWidth,
      this.circuitHeight);

    const position = this.circuitPositionRef.current;
    this.drawCar(circuitMarginX + (position[0] * this.scaleX), circuitMarginY + (position[1] * this.scaleY));
  }

  this.drawCar = (x, y) => {
    this.ctx.beginPath();
    this.ctx.arc(x, y, 3 * this.scaleX, 0, 2 * Math.PI, false);
    this.ctx.lineWidth = 1.5;
    this.ctx.strokeStyle = "#666666";
    this.ctx.stroke();
    this.ctx.fillStyle = "#FF0000";
    this.ctx.fill();
  }

  return {
    init: this.init,
    run: this.run,
    stop: this.stop
  }
}

export default UpdateRenderer;
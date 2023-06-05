function UpdateRenderer() {
  const circuitReferenceWidth = 300;
  const circuitReferenceHeight = 150;
  const circuitScaleX = 0.5;
  const circuitScaleY = 0.5;
  const circuitMarginX = 16;
  const circuitMarginY = 16;

  const cameraImage = new Image();
  const circuitImage = new Image();

  let currentTick = null;
  let circuitImageLoaded = false;

  this.init = (width, height, canvas, updateRef, circuitImageSrc) => {
    this.ctx = canvas.getContext("2d");
    this.canvas = canvas;
    this.updateRef = updateRef;

    this.width = width;
    this.height = height;
    this.circuitWidth = this.width * circuitScaleX - 2 * circuitMarginX;
    this.circuitHeight = this.height * circuitScaleY - 2 * circuitScaleY;

    this.scaleX = this.circuitWidth / circuitReferenceWidth;
    this.scaleY = this.circuitHeight / circuitReferenceHeight;

    cameraImage.onload = this.loadCameraImage;

    circuitImage.onload = () => (circuitImageLoaded = true);
    circuitImage.src = circuitImageSrc;
  };

  this.run = () => {
    currentTick = requestAnimationFrame(this.tick);
  };

  this.stop = () => {
    if (currentTick != null) {
      cancelAnimationFrame(currentTick);
    }
  };

  this.tick = () => {
    if (this.updateRef.current.image) {
      const image = JSON.parse(this.updateRef.current.image);
      cameraImage.src = `data:image/png;base64,${image.image}`;
    } else {
      currentTick = requestAnimationFrame(this.tick);
    }
  };

  this.loadCameraImage = () => {
    // clear canvas
    this.ctx.canvas.width = this.width;
    this.ctx.canvas.height = this.height;

    this.ctx.drawImage(cameraImage, 0, 0, this.width, this.height);

    // Draw circuit image and car position point
    this.drawAerialView();

    currentTick = requestAnimationFrame(this.tick);
  };

  this.drawAerialView = () => {
    if (circuitImageLoaded) {
      this.ctx.drawImage(
        circuitImage,
        circuitMarginX,
        circuitMarginY,
        this.circuitWidth,
        this.circuitHeight
      );

      if (this.updateRef.current.map) {
        const position = this.updateRef.current.map
          .slice(1)
          .slice(0, -1)
          .split(",");
        this.drawCar(
          circuitMarginX + position[0] * this.scaleX,
          circuitMarginY + position[1] * this.scaleY
        );
      }

      if (this.updateRef.current.lap) {
        this.drawLapTime(this.updateRef.current.lap);
      }
    }
  };

  this.drawCar = (x, y) => {
    this.ctx.beginPath();
    this.ctx.arc(x, y, 3 * this.scaleX, 0, 2 * Math.PI, false);
    this.ctx.lineWidth = 1.5;
    this.ctx.strokeStyle = "#666666";
    this.ctx.stroke();
    this.ctx.fillStyle = "#FF0000";
    this.ctx.fill();
  };

  this.drawLapTime = (time) => {
    this.ctx.font = "bold 24px sans-serif";
    this.ctx.textAlign = "right";
    this.ctx.textBaseline = "top";
    this.ctx.fillText(time, this.ctx.canvas.width - 16, 16);
  };

  return {
    init: this.init,
    run: this.run,
    stop: this.stop,
  };
}

export default UpdateRenderer;

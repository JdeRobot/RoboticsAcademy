import { CIRCUITS_CHECKPOINTS } from "./FollowLineChekpoints";

function UpdateRenderer() {
  const circuitReferenceWidth = 300;
  const circuitReferenceHeight = 150;
  const circuitScaleX = 0.3;
  const circuitScaleY = 0.3;
  const circuitMarginX = 16;
  const circuitMarginY = 16;

  const cameraImage = new Image();
  const circuitImage = new Image();

  let isRunning = false;
  let currentTick = null;
  let circuitImageLoaded = false;
  let startTime = null;
  let n = 0;

  this.init = (width, height, canvas, updateRef, circuitImageSrc, world) => {
    this.ctx = canvas.getContext("2d");
    this.canvas = canvas;
    this.updateRef = updateRef;

    this.width = width;
    this.height = height;
    this.percent = 0;
    this.circuitWidth = this.width * circuitScaleX - 2 * circuitMarginX;
    this.circuitHeight = this.height * circuitScaleY - 2 * circuitScaleY;

    this.scaleX = this.circuitWidth / circuitReferenceWidth;
    this.scaleY = this.circuitHeight / circuitReferenceHeight;

    cameraImage.onload = this.loadCameraImage;

    circuitImage.onload = () => (circuitImageLoaded = true);
    circuitImage.src = circuitImageSrc;
    this.world = world.toLowerCase();
  };

  this.run = () => {
    isRunning = true;
    startTime = new Date();
    currentTick = requestAnimationFrame(this.tick);
  };

  this.stop = () => {
    isRunning = false;
    this.ctx.clearRect(0, 0, this.width, this.height);

    this.percent = 0;
    n = 0;
    if (currentTick != null) {
      cancelAnimationFrame(currentTick);
    }
    this.drawProgressBar();
    this.drawLapTime();
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
        this.drawProgressBar();
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

  this.drawLapTime = () => {
    this.ctx.font = "bold 24px sans-serif";
    this.ctx.fillStyle = "red"; // Text color
    this.ctx.textAlign = "right";
    this.ctx.textBaseline = "top";

    if (isRunning) {
      const now = new Date();
      const elapsedTime = now - startTime;
      const seconds = (elapsedTime / 1000).toFixed(2);

      this.ctx.fillText(seconds + "s", this.ctx.canvas.width - 16, 16);
    } else {
      const clearWidth = 100;
      const clearHeight = 30;
      this.ctx.clearRect(
        this.ctx.canvas.width - clearWidth - 16,
        16,
        clearWidth,
        clearHeight
      );
      this.ctx.fillText(0 + "s", this.ctx.canvas.width - 16, 16);
    }
  };

  this.evaluator = (content) => {
    let checkpoints = CIRCUITS_CHECKPOINTS[this.world];
    var x = Math.round(content[0]);
    var y = Math.round(content[1]);

    var d = Math.sqrt(
      Math.pow(checkpoints[n][1] - x, 2) + Math.pow(checkpoints[n][2] - y, 2)
    );
    var d1 = Math.sqrt(
      Math.pow(checkpoints[n][1] + 5 - x, 2) +
        Math.pow(checkpoints[n][2] + 5 - y, 2)
    );
    var d2 = Math.sqrt(
      Math.pow(checkpoints[n][1] - 5 - x, 2) +
        Math.pow(checkpoints[n][2] - 5 - y, 2)
    );
    var d3 = Math.sqrt(
      Math.pow(checkpoints[n][1] + 5 - x, 2) +
        Math.pow(checkpoints[n][2] - 5 - y, 2)
    );
    var d4 = Math.sqrt(
      Math.pow(checkpoints[n][1] - 5 - x, 2) +
        Math.pow(checkpoints[n][2] + 5 - y, 2)
    );

    var threshold = 5;
    if (
      d <= threshold ||
      d1 <= threshold ||
      d2 <= threshold ||
      d3 <= threshold ||
      d4 <= threshold
    ) {
      n = n + 1;

      if (n >= checkpoints.length) {
        this.percent = 0;
      }
    }
    var progress_bar = Math.round((n / checkpoints.length) * 100);

    if (n >= checkpoints.length && progress_bar === 0) {
      this.percent = 100;
    } else {
      this.percent = progress_bar;
    }
  };

  this.drawProgressBar = () => {
    if (isRunning && this.updateRef.current.map) {
      let pose = this.updateRef.current.map.substring(
        1,
        this.updateRef.current.map.length - 1
      );
      let content = pose.split(",").map(function (item) {
        return parseFloat(item);
      });
      this.evaluator(content);
    }
    // Bar dimensions
    const barHeight = 25;
    const barWidth = this.width - 2 * circuitMarginX;

    // Bar position
    const barX = circuitMarginX;
    const barY = this.height - barHeight - circuitMarginY;

    // Calculate progress width
    const progressWidth = this.percent;
    if (this.ctx) {
      // Draw the full bar (background)
      this.ctx.fillStyle = "#CCCCCC";
      this.ctx.fillRect(barX, barY, barWidth, barHeight);

      // Draw the progress on the bar
      this.ctx.fillStyle = "#0066FF";
      this.ctx.fillRect(
        barX,
        barY,
        (progressWidth * barWidth) / 100,
        barHeight
      );

      // Add text to the progress bar
      this.ctx.fillStyle = "#000000"; // Text color
      this.ctx.font = "bold 20px sans-serif"; // Font size and style
      this.ctx.textAlign = "center"; // Horizontal alignment
      this.ctx.textBaseline = "middle"; // Vertical alignment
      const percentText = `${Math.round(this.percent)}%`; // Text to display
      this.ctx.fillText(percentText, barX + barWidth / 2, barY + barHeight / 2);
    }
  };

  return {
    init: this.init,
    run: this.run,
    stop: this.stop,
  };
}

export default UpdateRenderer;

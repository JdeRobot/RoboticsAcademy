import * as React from 'react';
import classNames from 'classnames';
import "./CanvasBirdEye.css"

export default function CanvasBirdEyeTest(props) {
  // "default" is the default circuit if not specified
  const {
    circuit = "default"
  } = props;

  // Canvas classes
  const canvasClass = classNames({
    "birds-eye": true
  }, circuit);

  function drawCircle(x, y) {
    ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

    ctx.beginPath();
    ctx.arc(x, y, 2, 0, 2 * Math.PI);
    ctx.closePath();

    ctx.lineWidth = 1.5;
    ctx.strokeStyle = "#666666";
    ctx.stroke();

    ctx.fillStyle = "#FF0000";
    ctx.fill();
  }

  React.useEffect(() => {
    var c = document.getElementById("birds-eye");
    var ctx = c.getContext("2d");
  }, []);

  return (
    <canvas
        id="birds-eye"
        className={canvasClass}
        width="200"
        height="100"
    >
      Your browser does not support the HTML canvas tag.
    </canvas>
  )
}
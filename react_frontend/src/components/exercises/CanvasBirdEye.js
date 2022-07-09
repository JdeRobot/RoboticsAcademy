import * as React from 'react';
export default function CanvasBirdEye(){
    function drawCircle(x, y){
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
}
    React.useEffect(() => {
        var c = document.getElementById("birds-eye");
        var ctx = c.getContext("2d");
    }, []);

    return (
        <canvas
            id="birds-eye"
            width="200"
            height="100"
            style={{ marginX:20 ,border: "2px solid #d3d3d3", backgroundRepeat: "no-repeat",height: 350, width: 340, }}
        >
            Your browser does not support the HTML canvas tag.
        </canvas>
    )
}

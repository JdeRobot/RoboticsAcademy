import { createContext, useState } from "react";

const BirdEyeContext = createContext();

export function BirdEyeProvider({ children }){
    let [ initialPosition, setInitialPosition] = useState(null);

    function drawCircle(x,y,ctx, mapCanvas){
        if (initialPosition == null){
            initialPosition = [x,y];
        }
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
    function drawInitialPosition(ctx,mapCanvas) {
	drawCircle(initialPosition[0], initialPosition[1],ctx,mapCanvas);
    }
    return(
    	<BirdEyeContext.Provider value={{ drawCircle, drawInitialPosition }}>{children}</BirdEyeContext.Provider>
	);
}

export default BirdEyeContext;
import { createContext, useState } from "react";

const ExerciseContext = createContext();

export function ExerciseProvider({ children }){
    const [ initialPosition, setInitialPosition] = useState(null);

    function drawCircle(x,y,ctx, mapCanvas){
        if (initialPosition == null){
            initialPosition = [x,y];
        }
        ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

	    cursor_x = x;
	    cursor_y = y;

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
    	<ExerciseContext.Provider value={{ }}>{children}</ExerciseContext.Provider>
	);
}

export default ExerciseContext;
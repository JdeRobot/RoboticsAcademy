import { createContext, useState } from "react";
import * as React from "react";

const CircuitSelectorContext = createContext();

export function CircuitSelectorProvider({ children }){
    const [ circuit, setCircuit] = useState('default');
    const [ backgroundImage, setBackgroundImage ] = React.useState("/static/exercises/follow_line_react/img/map.jpg");
    const getCircuitValue = () => {
        return circuit;
    }
    function handleChange(e,circuitSelector) {

        // let circuit = circuitSelector.current.options[circuit_selector.selectedIndex].value;
        let circuit_ = e.target.value;
        let bImgSrc = "/static/exercises/follow_line_react/img/map.jpg";
        circuitSelector.current.value = circuit_;
        setCircuit(circuit_);
        switch (circuit_) {
            case 'montreal': bImgSrc = "/static/exercises/follow_line_react/img/montreal.jpg";
                setBackgroundImage("/static/exercises/follow_line_react/img/montreal.jpg");
                             break;
            case 'montmelo': bImgSrc = "/static/exercises/follow_line_react/img/montmelo.jpg";
                setBackgroundImage("/static/exercises/follow_line_react/img/montmelo.jpg");
                             break;
            case 'nbg': bImgSrc = "/static/exercises/follow_line_react/img/nbg.jpg";
                setBackgroundImage("/static/exercises/follow_line_react/img/nbg.jpg");
                        break;
            default : bImgSrc = "/static/exercises/follow_line_react/img/map.jpg";
                setBackgroundImage("/static/exercises/follow_line_react/img/map.jpg");
        }

        let mapCanvas = document.getElementById("birds-eye");
        let ctx = mapCanvas.getContext("2d");
        ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
        let background = new Image();
        background.src = bImgSrc;
        console.log(background.src);
        // Make sure the image is loaded first otherwise nothing will draw.
        background.onload = function(){
            scaleToFit(background,ctx,mapCanvas);
        }

    // Disable connection button
        document.getElementById("connection-button").prop('disabled',true);
    // Set birds-eye background
    //     let canvas = document.querySelector('#birds-eye');

    // Set variable to toggle gazebo
    //     gazeboToggle = true;
    // // Stop the simulation
    //     stop();
    // // Kill actual sim
    //     startSim(2)
    // // StartSim
    //     startSim(1, circuit);
    //     alert('Loading circuit. Please wait until the connection is restored.');
    }

    function scaleToFit(img,ctx,canvas){
    // get the scale
        var scale = Math.min(canvas.width / img.width, canvas.height / img.height);
    // get the top left position of the image
        var x = (canvas.width / 2) - (img.width / 2) * scale;
        var y = (canvas.height / 2) - (img.height / 2) * scale;
        ctx.drawImage(img, x, y, img.width * scale, img.height * scale);
    }

    return(
    	<CircuitSelectorContext.Provider value={{ circuit, backgroundImage, scaleToFit, handleChange, getCircuitValue }}>{children}</CircuitSelectorContext.Provider>
	);
}

export default CircuitSelectorContext;
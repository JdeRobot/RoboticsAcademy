import * as React from 'react';
import {Box, Typography} from "@mui/material";
import CircuitSelectorContext from "../../contexts/CircuitSelectorContext";
export default function CanvasBirdEye(){
        const { backgroundImage, scaleToFit } = React.useContext(CircuitSelectorContext);
        const BirdsEye = React.useRef(null);
        React.useEffect(() => {
        // Set Background Image for the first time
        let mapCanvas = BirdsEye.current;

        let ctx = mapCanvas.getContext("2d");
        ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
        let background = new Image();
        background.src = backgroundImage;
        console.log(background.src);
        // Make sure the image is loaded first otherwise nothing will draw.
        background.onload = function(){
            scaleToFit(background,ctx,mapCanvas);
        }
    }, []);
    return (
        <Box sx={{m:3,p:2, display:"inline-flex",flexDirection:"column",border: "2px solid #d3d3d3"}}>
            <Typography align={'center'}>
                Visualization
            </Typography>
        <canvas
            id="birds-eye"
            ref={BirdsEye}
            height={500}
            width={500}
            sx={{ marginX:10 ,border: "2px solid #d3d3d3", backgroundSize: 100 ,backgroundRepeat: "no-repeat" }}
        >
            Your browser does not support the HTML canvas tag.
        </canvas>
            <div>
                <img id="gui_canvas"></img>
            </div>
            </Box>

    )
}

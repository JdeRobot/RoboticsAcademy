import * as React from 'react';
import {Box, Typography} from "@mui/material";
export default function CanvasBirdEye(){

    React.useEffect(() => {
        var mapCanvas = document.getElementById("birds-eye");
        var ctx = mapCanvas.getContext("2d");
    }, []);

    return (
        <Box sx={{m:3,p:2, display:"inline-flex",flexDirection:"column",border: "2px solid #d3d3d3"}}>
            <Typography>
                Visualization
            </Typography>
        <canvas
            id="birds-eye"
            width="200"
            height="100"
            style={{ marginX:20 ,border: "2px solid #d3d3d3", backgroundImage: `url("/static/exercises/follow_line_react/img/map.jpg")` ,backgroundRepeat: "no-repeat",height: 350, width: 340, }}
        >
            Your browser does not support the HTML canvas tag.
        </canvas>
            </Box>
    )
}

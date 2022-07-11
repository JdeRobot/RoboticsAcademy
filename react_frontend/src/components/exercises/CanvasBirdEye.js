import * as React from 'react';
export default function CanvasBirdEye(){

    React.useEffect(() => {
        var mapCanvas = document.getElementById("birds-eye");
        var ctx = mapCanvas.getContext("2d");
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

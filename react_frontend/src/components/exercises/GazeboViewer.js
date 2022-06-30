import {VncScreen} from "react-vnc";
import * as React from "@types/react";

function GazeboViewer() {
    return (
        <VncScreen
            url='ws://your-vnc-url.com'
            scaleViewport
            background="#000000"
            style={{
                width: '75vw',
                height: '75vh',
            }}

        />
    );
}

export default GazeboViewer;
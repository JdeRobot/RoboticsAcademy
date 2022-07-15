import {VncScreen} from "react-vnc";
import * as React from 'react';
import {Box, Typography} from "@mui/material";

function VncConsoleViewer() {
    return (
        <Box><Typography>
                Console 
            </Typography>
        <iframe
            src="http://127.0.0.1:1108/vnc.html?resize=remote&autoconnect=true"
            id={"console-vnc"}
            // scaleViewport
            // background="#000000"
            style={{
                width: '75vw',
                height: '75vh',
            }}

        />
            </Box>
    );
}

export default VncConsoleViewer;
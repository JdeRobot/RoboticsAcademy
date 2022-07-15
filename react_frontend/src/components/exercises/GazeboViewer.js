import * as React from 'react';
import {Box, Typography} from "@mui/material";

function GazeboViewer() {
    return (
        <Box>
            <Typography>
                Gazebo
            </Typography>
        <iframe
            src='http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true'
            id={"iframe"}
            style={{
                width: '75vw',
                height: '75vh',
            }}

        />
            </Box>
    );
}

export default GazeboViewer;
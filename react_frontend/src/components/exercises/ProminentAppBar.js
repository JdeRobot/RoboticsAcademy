import * as React from 'react';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import IconButton from '@mui/material/IconButton';
import Image from 'mui-image';
import {Box, Button, ButtonGroup, Typography} from "@mui/material";
import ConnectingAirportsIcon from '@mui/icons-material/ConnectingAirports';
import LaunchIcon from '@mui/icons-material/Launch';
import HelpCenterOutlinedIcon from '@mui/icons-material/HelpCenterOutlined';
import SchoolOutlinedIcon from '@mui/icons-material/SchoolOutlined';
import CodeOutlinedIcon from '@mui/icons-material/CodeOutlined';
import CommentOutlinedIcon from '@mui/icons-material/CommentOutlined';

import RoboticsTheme from '../RoboticsTheme.js';
function ProminentAppBar() {
    const connectionButton = React.useRef(null);
    React.useEffect(()=> {
         const onPageLoad = () => {
             // startSim(0,"default");
             // console.log(ws_manager);
             // const connectionButton = document.getElementById("connection-button");
             console.log(`Connection button --> ${connectionButton.current}`);
         };
         const onUnload = () => {
                 // startSim(2);
         };
         if(document.readyState == "complete"){
             onPageLoad();
         }else{
             window.addEventListener("load",onPageLoad);

             return ()=> window.removeEventListener("load",onPageLoad);
         }
     },[]);

    return (
        <RoboticsTheme>
        <AppBar position="static">
                <Toolbar sx={{ display: "flex", flexWrap: 'wrap', justifyContent:'space-between'}} >
                    <Box sx={{ display: "inline-flex", justifyContent:'space-between',alignItems: 'center'}}>
                    <Image src="/static/common/img/logo.gif" fit={"cover"}  width={50}></Image>
                        <Button id={"connection-button"} ref={connectionButton} startIcon={<ConnectingAirportsIcon/>} variant="contained" color={"success"} sx={{marginX:1 ,}} size={"small"}>Connect</Button>
                        <Button id={"launch-button"} startIcon={<LaunchIcon/>} variant="contained" color={"secondary"} sx={{marginX:1 ,}} size={"small"}>Launch</Button>
                        <IconButton aria-label="helpCenter" sx={{marginX:1 ,}}>
                            <HelpCenterOutlinedIcon />
                        </IconButton>
                    </Box>
                    <Typography variant="h5">
                        Follow Line exercise
                    </Typography>
                    <ButtonGroup variant="contained" color={"success"}>
                        <IconButton >
                            <SchoolOutlinedIcon />
                        </IconButton>
                        <IconButton >
                            <CodeOutlinedIcon />
                        </IconButton>
                        <IconButton >
                            <CommentOutlinedIcon />
                        </IconButton>
                    </ButtonGroup>
                </Toolbar>
        </AppBar>
        </RoboticsTheme>
    );
}

export default ProminentAppBar;
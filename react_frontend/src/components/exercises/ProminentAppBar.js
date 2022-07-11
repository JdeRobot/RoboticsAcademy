import * as React from 'react';
import AppBar from '@mui/material/AppBar';
import List from '@mui/material/List';
import Toolbar from '@mui/material/Toolbar';
import IconButton from '@mui/material/IconButton';
import Image from 'mui-image';
import {Box, Button, ButtonGroup, Divider, Input, TextField, Typography} from "@mui/material";
import ConnectingAirportsIcon from '@mui/icons-material/ConnectingAirports';
import LaunchIcon from '@mui/icons-material/Launch';
import HelpCenterOutlinedIcon from '@mui/icons-material/HelpCenterOutlined';
import SchoolOutlinedIcon from '@mui/icons-material/SchoolOutlined';
import CodeOutlinedIcon from '@mui/icons-material/CodeOutlined';
import CommentOutlinedIcon from '@mui/icons-material/CommentOutlined';
import CloudUploadOutlinedIcon from '@mui/icons-material/CloudUploadOutlined';
import SmartToyOutlinedIcon from '@mui/icons-material/SmartToyOutlined';
import PlayCircleOutlineOutlinedIcon from '@mui/icons-material/PlayCircleOutlineOutlined';
import RestartAltOutlinedIcon from '@mui/icons-material/RestartAltOutlined';
import VrpanoOutlinedIcon from '@mui/icons-material/VrpanoOutlined';
import TerminalOutlinedIcon from '@mui/icons-material/TerminalOutlined';
import VideogameAssetOutlinedIcon from '@mui/icons-material/VideogameAssetOutlined';
import SaveIcon from '@mui/icons-material/Save';
import RoboticsTheme from '../RoboticsTheme.js';
function ProminentAppBar() {
    // React.useEffect(()=> {
    //     const onPageLoad = () => {
    //         startSim(0,"default");
    //         console.log(ws_manager);
    //         $("#connection-button").prop('disabled',true);
    //     };
    //     const onUnload = () => {
    //             startSim(2);
    //     };
    //     if(document.readyState == "complete"){
    //         onPageLoad();
    //     }else{
    //         window.addEventListener("load",onPageLoad);
    //
    //         return ()=> window.removeEventListener("load",onPageLoad);
    //     }
    // },[]);
    return (
        <RoboticsTheme>
        <AppBar position="static">
                <Toolbar sx={{ display: "flex", flexWrap: 'wrap', justifyContent:'space-between'}} >
                    <Box sx={{ display: "inline-flex", justifyContent:'space-between',alignItems: 'center'}}>
                    <Image src="/static/common/img/logo.gif" fit={"cover"}  width={50}></Image>
                        <Button id={"connection-button"} startIcon={<ConnectingAirportsIcon/>} variant="contained" color={"success"} sx={{marginX:1 ,}} size={"small"}>Connect</Button>
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
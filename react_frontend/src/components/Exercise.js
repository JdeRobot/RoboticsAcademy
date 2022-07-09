import React from 'react';
import '../styles/Exercise.css';
// import ProminentAppBar from "./exercises/ProminentAppBar";
import RoboticsTheme from './RoboticsTheme';
import Toolbar from "@mui/material/Toolbar";
import {Box, Button, ButtonGroup, Divider, Input, TextField, Typography} from "@mui/material";
import Image from "mui-image";
import ConnectingAirportsIcon from "@mui/icons-material/ConnectingAirports";
import LaunchIcon from "@mui/icons-material/Launch";
import IconButton from "@mui/material/IconButton";
import HelpCenterOutlinedIcon from "@mui/icons-material/HelpCenterOutlined";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";
import SaveIcon from "@mui/icons-material/Save";
import SmartToyOutlinedIcon from "@mui/icons-material/SmartToyOutlined";
import PlayCircleOutlineOutlinedIcon from "@mui/icons-material/PlayCircleOutlineOutlined";
import RestartAltOutlinedIcon from "@mui/icons-material/RestartAltOutlined";
import VrpanoOutlinedIcon from "@mui/icons-material/VrpanoOutlined";
import TerminalOutlinedIcon from "@mui/icons-material/TerminalOutlined";
import VideogameAssetOutlinedIcon from "@mui/icons-material/VideogameAssetOutlined";
import AppBar from "@mui/material/AppBar";
import CircuitSelector from "./exercises/CircuitSelector";
import CanvasBirdEye from "./exercises/CanvasBirdEye";
import AceEditorRobot from "./exercises/AceEditorRobot";

function Exercise() {
    const [launchLevel, setLaunchLevel] = React.useState(0);
    const [theoryMode, setTheoryMode] = React.useState(false);
    const [codeMode, setCodeMode] = React.useState(true);
    const [forumMode, setForumMode] = React.useState(false);
    var ws_manager;
    const [ firstAttempt, setFirstAttempt ] = React.useState(true);
    const [sendCode, setSendCode ] = React.useState(false);
    const [simResume, setSimResume ] = React.useState(false);
    const [simStop, setSimStop ] = React.useState(false);
    const [simReset, setSimReset ] = React.useState(false);
    const [gazeboOn, setGazeboOn] = React.useState(false);
    const [gazeboToggle, setGazeboToggle] = React.useState(false);
    var websocket_address = "127.0.0.1";
    function startSim(step, circuit="default") {
    var level = 0;
    let websockets_connected = false;
    if (step == 0) {
        // setWsManager(new WebSocket("ws://" + websocket_address + ":8765/"));
        ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    }
    else if (step == 1) {
        radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        var size = get_novnc_size();
        console.log(circuit);
        ws_manager.send(JSON.stringify({
            "command": "open", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString(), "circuit": circuit}));
        level++;
        radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        ws_manager.send(JSON.stringify({"command" : "Pong"}));
    }
    else if (step == 2) {
        ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
    }

    ws_manager.onopen = function (event) {
        level++;
        radiConect.contentWindow.postMessage({connection: 'manager', command: 'up'}, '*');
        radiConect.contentWindow.postMessage({connection: 'exercise', command: 'available'}, '*');
    }

    ws_manager.onmessage = function (event) {
        //console.log(event.data);
        if (event.data.level > level) {
            level = event.data.level;
            radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        }
        if (event.data.includes("Ping")) {
            if (!websockets_connected && event.data == "Ping3") {
                level = 4;
                radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
                websockets_connected = true;
                declare_code(websocket_address);
                declare_gui(websocket_address);
            }
            if (gazeboToggle) {
                console.log("toggle gazebo");
                if (gazeboOn) {
                    ws_manager.send(JSON.stringify({"command" : "startgz"}));
                } else {
                    ws_manager.send(JSON.stringify({"command" : "stopgz"}));
                }

                setGazeboToggle(false);
            }else if (sendCode){
                let python_code = editor.getValue();
		        python_code = "#code\n" + python_code;
                ws_manager.send(JSON.stringify({"command": "evaluate", "code": python_code}));
                setSendCode(false);
            }else if (simReset){
                console.log("reset simulation");
                ws_manager.send(JSON.stringify({"command": "reset"}));
                setSimReset(false);
            } else if (simStop){
                ws_manager.send(JSON.stringify({"command": "stop"}));
                setSimStop(false);
            } else if (simResume){
                ws_manager.send(JSON.stringify({"command": "resume"}));
                setSimResume(false);
            } else {
                setTimeout(function () {
                    ws_manager.send(JSON.stringify({"command" : "Pong"}));
                }, 1000)
            }
        }
        if (event.data.includes("evaluate")) {
            if (event.data.length < 9) {    // If there is an error it is sent along with "evaluate"
                submitCode();
            } else {
                let error = event.data.substring(10,event.data.length);
                radiConect.contentWindow.postMessage({connection: 'exercise', command: 'error', text: error}, '*');
                toggleSubmitButton(true);
            }
            setTimeout(function () {
                ws_manager.send(JSON.stringify({"command" : "Pong"}));
            }, 1000)
        } else if (event.data.includes("PingDone")) {
            enablePlayPause(true);
            toggleResetButton(true);
            if (resetRequested == true) {
                togglePlayPause(false);
                resetRequested = false;
            }
        }
        }
    }
    function toggleGazebo() {
    if (gazeboOn) {
        setGazeboOn(false);
    } else {
        setGazeboOn(true);
    }

    setGazeboToggle(true);
    }

    function resetSimulation() {
        setSimReset(true);
    }

    function stopSimulation() {
        setSimStop(true);
    }

    function resumeSimulation() {
        setSimResume(true);
    }

    function checkCode() {
        setSendCode(true);
    }
    React.useEffect(()=> {
        const onPageLoad = () => {
            startSim(0,"default");
            console.log(ws_manager);
            $("#connection-button").prop('disabled',true);
        };
        const onUnload = () => {
                startSim(2);
        };
        if(document.readyState == "complete"){
            onPageLoad();
        }else{
            window.addEventListener("load",onPageLoad);

            return ()=> window.removeEventListener("load",onPageLoad);
        }
    },[]);
    function onClickTheory() {
        if(!theoryMode){
            setTheoryMode(true);
            setCodeMode(false);
            setForumMode(false);
        }
    }
    function onClickCode() {
        if(!codeMode){
            setTheoryMode(false);
            setCodeMode(true);
            setForumMode(false);
        }
    }
    function onClickForum() {
        if(!forumMode){
            setTheoryMode(false);
            setCodeMode(false);
            setForumMode(true);
        }
    }
    function getLaunchLevel() {
        return launchLevel;
    }
  return (
    <RoboticsTheme>
      <div className="Exercise">
        <AppBar position="static" sx={{marginBottom:3, marginTop: -1}}>
                <Toolbar sx={{ display: "flex", flexWrap: 'wrap', justifyContent:'space-between'}} >
                    <Box sx={{ display: "inline-flex", justifyContent:'space-between',alignItems: 'center'}}>
                    <Image src="/static/common/img/logo.gif" fit={"cover"}  width={50}></Image>
                    <ButtonGroup size={"small"}>
                        <Button id={"connection-button"} startIcon={<ConnectingAirportsIcon/>} variant="contained" color={"success"} sx={{marginX:1 ,}} size={"small"}>Connect</Button>
                        <Button id={"launch-button"} startIcon={<LaunchIcon/>} variant="contained" color={"secondary"} sx={{marginX:1 ,}} size={"small"}>Launch</Button>
                        <IconButton aria-label="helpCenter" sx={{marginX:1 ,}}>
                            <HelpCenterOutlinedIcon />
                        </IconButton>
                    </ButtonGroup>
                    </Box>
                    <Typography variant="h5">
                        Follow Line exercise
                    </Typography>
                    <ButtonGroup id={"Control"} variant="outlined" color={"success"} aria-label="outlined button group">
                        <IconButton id={"open-theory"} aria-label="documentation">
                            <SchoolOutlinedIcon />
                        </IconButton>
                        <IconButton id={"open-exercise"} aria-label="code">
                            <CodeOutlinedIcon />
                        </IconButton>
                        <IconButton id={"open-forum"} aria-label="forum">
                            <CommentOutlinedIcon />
                        </IconButton>
                    </ButtonGroup>
                </Toolbar>
                <Divider color={"secondary"}  variant="middle" sx={{m:0.5,marginX:2,marginTop:-0.5}}/>
                <Toolbar id={"exercise-controls"} sx={{ display: "flex", flexWrap: 'wrap' ,justifyContent:'space-between',alignItems: 'center'}} >
                    <label  htmlFor="contained-button-file" sx={{ padding: 0 ,display: 'none'}}>
                            <Input accept=".py" id="contained-button-file" multiple type="file" sx ={{display : 'none'}}/>
                            <Button id={"load"} variant="contained" color={"secondary"} startIcon={<CloudUploadOutlinedIcon/>} sx={{m:1}}>
                                Load file
                            </Button>
                        </label>
                        <Button id={"save"} variant="contained" color={"secondary"}  startIcon={<SaveIcon/>} sx ={{m:1}}>
                            Save file
                        </Button>

                        <Button id={"loadIntoRobot"} color={"secondary"} startIcon={<SmartToyOutlinedIcon/>} sx ={{m:0.5}} variant={"outlined"} >
                            Load in robot
                        </Button>
                        <Button id={"submit"} color={"secondary"}  startIcon={<PlayCircleOutlineOutlinedIcon/>} sx ={{m:0.5}} variant={"outlined"} >
                            Play
                        </Button>
                        <Button id={"stop"} color={"secondary"}  startIcon={<RestartAltOutlinedIcon/>}  sx ={{m:0.5}} variant={"outlined"} >
                            reset
                        </Button>

                        <TextField
                            id="standard-number"
                            label="Brain Freq (Hz)"
                            type="number"
                            defaultValue={1}
                            size={"small"}
                            sx ={{width:160, m:1}}
                            inputProps={{ min: 0, max: 30,onKeyDown: (event) => {
                                    event.preventDefault();
                                },  }}
                            color = {"secondary"}
                        />
                        <TextField
                            id="standard-number"
                            label="GUI Freq (Hz)"
                            defaultValue={1}
                            type="number"
                            sx ={{width:160, m:1}}
                            inputProps={{  min: 0, max: 10 ,onKeyDown: (event) => {
                                    event.preventDefault();
                                },}}
                            size={"small"}
                            color = {"secondary"}
                        />
                        <TextField
                            id={"standard-input"}
                            defaultValue="Sim RTF: 0"
                            color = {"secondary"}
                            InputProps={{
                                readOnly: true,
                            }}
                            size={"small"}
                            sx ={{width:120, m:1}}

                        />
                        <Button size={"medium"} variant="contained" color={"secondary"} component="span" sx ={{m:1}} startIcon={<VrpanoOutlinedIcon/>} >
                            View Sim
                        </Button>
                        <Button size={"medium"} variant="contained" color={"secondary"} component="span" sx ={{m:1}} startIcon={<TerminalOutlinedIcon/>} >
                            View Console
                        </Button>
                        <Button size={"medium"} variant="contained" color={"secondary"} component="span" sx ={{m:1}} startIcon={<VideogameAssetOutlinedIcon/>} >
                            Teleoperate
                        </Button>
                </Toolbar>
        </AppBar>
          {/*<Box sx={{display:"flex" ,flexWrap: 'wrap' }}>*/}
          {/*    <AceEditorRobot/>*/}
          {/*    <Box sx={{display:"flex" , flexDirection:"column" ,flexWrap: 'wrap' }}>*/}
          {/*    <CircuitSelector />*/}
          {/*    /!*<Divider />*!/*/}
          {/*    <CanvasBirdEye />*/}
          {/*</Box>*/}
          {/*</Box>*/}
      </div>
    </RoboticsTheme>
  );
}

export default Exercise;
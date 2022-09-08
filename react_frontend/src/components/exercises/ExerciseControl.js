import * as React from "react";
import Toolbar from "@mui/material/Toolbar";
import { Box, Button, TextField, Fab } from "@mui/material";
import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";
import SmartToyOutlinedIcon from "@mui/icons-material/SmartToyOutlined";
import PlayCircleOutlineOutlinedIcon from "@mui/icons-material/PlayCircleOutlineOutlined";
import RestartAltOutlinedIcon from "@mui/icons-material/RestartAltOutlined";
import VrpanoOutlinedIcon from "@mui/icons-material/VrpanoOutlined";
import TerminalOutlinedIcon from "@mui/icons-material/TerminalOutlined";
import VideogameAssetOutlinedIcon from "@mui/icons-material/VideogameAssetOutlined";
import StopCircleOutlinedIcon from "@mui/icons-material/StopCircleOutlined";
import SaveIcon from "@mui/icons-material/Save";
import RoboticsTheme from "../RoboticsTheme.js";
import ExerciseContext from "../../contexts/ExerciseContext";

function ExerciseControl() {
  const {
    onClickSave,
    check,
    resetSim,
    start,
    stop,
    loadFileButton,
    changeGzWeb,
    changeConsole,
    teleOpButtonClick,
    playState,
    codeFrequencyUpdate,
    guiFrequencyUpdate,
    guiFreq,
    brainFreq,
  } = React.useContext(ExerciseContext);
  return (
    <RoboticsTheme>
      <Toolbar
        sx={{
          display: "flex",
          flexWrap: "wrap",
          justifyContent: "space-between",
          alignItems: "center",
          m: 1,
          border: "2px solid #d3d3d3",
        }}
      >
        <Box>
          <Button
            variant="contained"
            sx={{ m: 1 }}
            color={"secondary"}
            startIcon={<CloudUploadOutlinedIcon />}
            component="label"
          >
            Load file
            <input hidden accept=".py" type="file" onChange={loadFileButton} />
          </Button>
          <Button
            id={"save"}
            variant="contained"
            color={"secondary"}
            onClick={onClickSave}
            startIcon={<SaveIcon />}
            sx={{ m: 1 }}
          >
            Save file
          </Button>
        </Box>
        <Box>
          <Button
            id={"loadIntoRobot"}
            color={"secondary"}
            onClick={check}
            startIcon={<SmartToyOutlinedIcon />}
            sx={{ m: 0.5 }}
            variant={"outlined"}
          >
            Load in robot
          </Button>
          {/*<Fab*/}
          {/*  id={"submit"}*/}
          {/*  color={playState ? "success" : "secondary"}*/}
          {/*  startIcon={*/}
          {/*    playState ? (*/}
          {/*      <PlayCircleOutlineOutlinedIcon />*/}
          {/*    ) : (*/}
          {/*      <StopCircleOutlinedIcon />*/}
          {/*    )*/}
          {/*  }*/}
          {/*  sx={{ m: 0.5 }}*/}
          {/*  onClick={playState ? start : stop}*/}
          {/*  // onClick={stop}*/}
          {/*  variant={"outlined"}*/}
          {/*>*/}
          {/*  {playState ? "Play" : "Stop"}*/}
          {/*</Fab>*/}
          <Button
            id={"reset"}
            color={"secondary"}
            startIcon={<RestartAltOutlinedIcon />}
            sx={{ m: 0.5 }}
            onClick={resetSim}
            variant={"outlined"}
          >
            Reset
          </Button>
        </Box>
        <Box>
          {/*<Slider*/}
          {/*  aria-label="Temperature"*/}
          {/*  defaultValue={30}*/}
          {/*  getAriaValueText={valuetext}*/}
          {/*  valueLabelDisplay="auto"*/}
          {/*  step={10}*/}
          {/*  marks*/}
          {/*  min={10}*/}
          {/*  max={110}*/}
          {/*/>*/}
          <TextField
            id={"code_freq"}
            label="Brain Freq (Hz)"
            type="number"
            value={brainFreq}
            size={"small"}
            title={
              "The brain frequency is the value that tells the robot with what frequency should the code be updated"
            }
            onChange={codeFrequencyUpdate}
            sx={{ width: 160, m: 1 }}
            inputProps={{
              min: 1,
              max: 30,
            }}
            color={"secondary"}
          />
          <TextField
            id={"gui_freq"}
            label="GUI Freq (Hz)"
            value={guiFreq}
            type="number"
            sx={{ width: 160, m: 1 }}
            onChange={guiFrequencyUpdate}
            title={
              "This value corresponds to the frequency the UI and visuals will be updated."
            }
            inputProps={{
              min: 1,
              max: 30,
            }}
            size={"small"}
            color={"secondary"}
          />
          {/*<TextField*/}
          {/*  label={"Sim RTF:"}*/}
          {/*  defaultValue={0}*/}
          {/*  type="number"*/}
          {/*  color={"secondary"}*/}
          {/*  InputProps={{*/}
          {/*    readOnly: true,*/}
          {/*  }}*/}
          {/*  title={*/}
          {/*    "This value is the real time factor of the simulation, it let us know the rate of time in the simulation and real time. As you get closer to 1, the difference between real time and simulation time decreases."*/}
          {/*  }*/}
          {/*  size={"small"}*/}
          {/*  sx={{ width: 120, m: 1 }}*/}
          {/*/>*/}
        </Box>
        <Box>
          <Button
            id={"gazebo_button"}
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
            title={"Activate the simulation Screen"}
            onClick={changeGzWeb}
            startIcon={<VrpanoOutlinedIcon />}
          >
            View Sim
          </Button>
          <Button
            id={"console_button"}
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
            title={"Open the console"}
            onClick={changeConsole}
            startIcon={<TerminalOutlinedIcon />}
          >
            View Console
          </Button>
          <Button
            id={"teleop_button"}
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
            onClick={teleOpButtonClick}
            title={"Use the arrow keys to operate the F1"}
            startIcon={<VideogameAssetOutlinedIcon />}
          >
            Teleoperate
          </Button>
        </Box>
      </Toolbar>
      <Fab
        id={"submit"}
        color={playState ? "success" : "secondary"}
        sx={{ m: 0.5, position: "fixed", bottom: 2, right: 2 }}
        onClick={playState ? start : stop}
        variant={"outlined"}
      >
        {playState ? (
          <PlayCircleOutlineOutlinedIcon />
        ) : (
          <StopCircleOutlinedIcon />
        )}
      </Fab>
    </RoboticsTheme>
  );
}

export default ExerciseControl;

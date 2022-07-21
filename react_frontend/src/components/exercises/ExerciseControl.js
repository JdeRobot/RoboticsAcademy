import * as React from "react";
import Toolbar from "@mui/material/Toolbar";
import { Box, Button, Input, TextField, Typography } from "@mui/material";
import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";
import SmartToyOutlinedIcon from "@mui/icons-material/SmartToyOutlined";
import PlayCircleOutlineOutlinedIcon from "@mui/icons-material/PlayCircleOutlineOutlined";
import RestartAltOutlinedIcon from "@mui/icons-material/RestartAltOutlined";
import VrpanoOutlinedIcon from "@mui/icons-material/VrpanoOutlined";
import TerminalOutlinedIcon from "@mui/icons-material/TerminalOutlined";
import VideogameAssetOutlinedIcon from "@mui/icons-material/VideogameAssetOutlined";
import SaveIcon from "@mui/icons-material/Save";
import RoboticsTheme from "../RoboticsTheme.js";
import ExerciseContext from "../../contexts/ExerciseContext";

function ExerciseControl() {
  const {
    onClickSave,
    check,
    resetSim,
    start,
    loadFileButton,
    changegzweb,
    changeconsole,
    guiFreqValue,
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
          <label htmlFor="contained-button-file">
            <Input
              accept=".py"
              id="contained-button-file"
              multiple
              type="file"
              sx={{ display: "none" }}
            />
            <Button
              id={"load"}
              variant="contained"
              color={"secondary"}
              startIcon={<CloudUploadOutlinedIcon />}
              onClick={loadFileButton}
              sx={{ m: 1 }}
            >
              Load file
            </Button>
          </label>
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
          <Button
            id={"submit"}
            color={"secondary"}
            startIcon={<PlayCircleOutlineOutlinedIcon />}
            sx={{ m: 0.5 }}
            onClick={start}
            // onClick={stop}
            variant={"outlined"}
          >
            Play
          </Button>
          <Button
            id={"stop"}
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
          <TextField
            id={"letters"}
            label="Brain Freq (Hz)"
            type="number"
            defaultValue={12}
            size={"small"}
            title={
              "The brain frequency is the value that tells the robot with what frequency should the code be updated"
            }
            sx={{ width: 160, m: 1 }}
            inputProps={{
              min: 1,
              max: 30,
              onKeyDown: (event) => {
                event.preventDefault();
              },
            }}
            color={"secondary"}
          />
          <TextField
            id="standard-number"
            label="GUI Freq (Hz)"
            defaultValue={1}
            type="number"
            sx={{ width: 160, m: 1 }}
            title={
              "This value corresponds to the frequency the UI and visuals will be updated."
            }
            inputProps={{
              min: 0,
              max: 10,
              onKeyDown: (event) => {
                event.preventDefault();
              },
            }}
            size={"small"}
            color={"secondary"}
          />
          <TextField
            id={"standard-input"}
            defaultValue="Sim RTF: 0"
            color={"secondary"}
            InputProps={{
              readOnly: true,
            }}
            title={
              "This value is the real time factor of the simulation, it let us know the rate of time in the simulation and real time. As you get closer to 1, the difference between real time and simulation time decreases."
            }
            size={"small"}
            sx={{ width: 120, m: 1 }}
          />
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
            onClick={changegzweb}
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
            onClick={changeconsole}
            startIcon={<TerminalOutlinedIcon />}
          >
            View Console
          </Button>
          <Button
            hidden
            id={"teleop_button"}
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
            title={"Use the arrow keys to operate the F1"}
            startIcon={<VideogameAssetOutlinedIcon />}
          >
            Teleoperate
          </Button>
        </Box>
      </Toolbar>
    </RoboticsTheme>
  );
}

export default ExerciseControl;

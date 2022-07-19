import * as React from "react";
import Toolbar from "@mui/material/Toolbar";
import { Box, Button, Input, TextField } from "@mui/material";
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
  const { onClickSave } = React.useContext(ExerciseContext);
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
            variant={"outlined"}
          >
            Play
          </Button>
          <Button
            id={"stop"}
            color={"secondary"}
            startIcon={<RestartAltOutlinedIcon />}
            sx={{ m: 0.5 }}
            variant={"outlined"}
          >
            reset
          </Button>
        </Box>
        <Box>
          <TextField
            id={"letters"}
            label="Brain Freq (Hz)"
            type="number"
            value={12}
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
            size={"small"}
            sx={{ width: 120, m: 1 }}
          />
        </Box>
        <Box>
          <Button
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
            startIcon={<VrpanoOutlinedIcon />}
          >
            View Sim
          </Button>
          <Button
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
            startIcon={<TerminalOutlinedIcon />}
          >
            View Console
          </Button>
          <Button
            hidden
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
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

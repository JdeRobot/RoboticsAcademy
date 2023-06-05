import * as React from "react";
import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";
import SmartToyOutlinedIcon from "@mui/icons-material/SmartToyOutlined";
import PlayCircleOutlineOutlinedIcon from "@mui/icons-material/PlayCircleOutlineOutlined";
import RestartAltOutlinedIcon from "@mui/icons-material/RestartAltOutlined";
import VrpanoOutlinedIcon from "@mui/icons-material/VrpanoOutlined";
import TerminalOutlinedIcon from "@mui/icons-material/TerminalOutlined";
import VideogameAssetOutlinedIcon from "@mui/icons-material/VideogameAssetOutlined";
import StopCircleOutlinedIcon from "@mui/icons-material/StopCircleOutlined";
import SaveIcon from "@mui/icons-material/Save";
import WebAssetIcon from "@mui/icons-material/WebAsset";
import PsychologyIcon from "@mui/icons-material/Psychology";
import Modal from "@mui/material/Modal";
import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";
import {
  Divider,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  Paper,
  TableRow,
  IconButton,
} from "@mui/material";
import CloseIcon from "@mui/icons-material/Close";
import PropTypes from "prop-types";

const style = {
  position: "absolute",
  top: "50%",
  left: "50%",
  transform: "translate(-50%, -50%)",
  width: 800,
  bgcolor: "background.paper",
  border: "2px solid #000",
  boxShadow: 24,
  p: 4,
};

const createData = (icon, name, description) => {
  return { icon, name, description };
};

const control = [
  createData(<CloudUploadOutlinedIcon />, "Load", "Load code from local files"),
  createData(<SaveIcon />, "Save", "Save your code "),
  createData(
    <SmartToyOutlinedIcon />,
    "Load in Robot",
    "Load your code into the robot"
  ),
  createData(<PlayCircleOutlineOutlinedIcon />, "Play", "Execute your code"),
  createData(<StopCircleOutlinedIcon />, "Pause", "Pause the simulation"),
  createData(<RestartAltOutlinedIcon />, "Reset", "Reset the simulation"),
  createData(<VrpanoOutlinedIcon />, "Gazebo", "Open gazebo web"),
  createData(<TerminalOutlinedIcon />, "Console", "Open terminal"),
  createData(
    <VideogameAssetOutlinedIcon />,
    "Teleoperate",
    "Operate the vehicle position manually"
  ),
  createData(
    <PsychologyIcon />,
    "Brain frequency",
    "Adjusts the running frequency of the code"
  ),
  createData(
    <WebAssetIcon />,
    "GUI frequency",
    "Adjusts the running frequency of the GUI"
  ),
];

const DenseTable = () => {
  return (
    <TableContainer component={Paper}>
      <Table
        sx={{
          minWidth: 650,
          backgroundColor: "#ffa726",
        }}
        size="small"
        aria-label="a dense table"
      >
        <TableHead>
          <TableRow>
            <TableCell>Icon</TableCell>
            <TableCell>Name </TableCell>
            <TableCell align="right">Description</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {control.map((row) => (
            <TableRow
              key={row.name}
              sx={{
                "&:last-child td, &:last-child th": { border: 0 },
                backgroundColor: "#329D9C",
              }}
            >
              <TableCell component="th" scope="row">
                {row.icon}
              </TableCell>
              <TableCell>{row.name}</TableCell>
              <TableCell align="right">{row.description}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );
};

const InfoModal = (props) => {
  const { openInfoModal, handleInfoModalClose } = React.useContext(
    props.context
  );
  return (
    <div>
      <Modal
        open={openInfoModal}
        onClose={handleInfoModalClose}
        aria-labelledby="modal-modal-title"
        aria-describedby="modal-modal-description"
      >
        <Box id={"control 1"} sx={style}>
          <Box
            sx={{
              display: "flex",
              justifyContent: "space-between",
              alignItems: "center",
            }}
          >
            <Typography id="modal-modal-title" variant="h6" component="h2">
              Controls
            </Typography>
            <IconButton onClick={handleInfoModalClose} color={"error"}>
              <CloseIcon />
            </IconButton>
          </Box>

          <Divider color={"secondary"} variant={"middle"} sx={{ m: 2 }} />
          <DenseTable />
        </Box>
      </Modal>
    </div>
  );
};

InfoModal.propTypes = {
  context: PropTypes.any,
};

export default InfoModal;

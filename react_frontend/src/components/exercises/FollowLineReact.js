import * as React from 'react';
import {Box} from "@mui/material";
import ExerciseControl from "./ExerciseControl";
import AceEditorRobot from "./AceEditorRobot";
import CircuitSelector from "./CircuitSelector";
import CanvasBirdEye from "./CanvasBirdEye";
import {CircuitSelectorProvider} from "../../contexts/CircuitSelectorContext";
import VncConsoleViewer from "./VncConsoleViewer";
import GazeboViewer from "./GazeboViewer";

function FollowLineReact() {

    return (
        <Box>
            <CircuitSelectorProvider>
            <ExerciseControl/>
            <AceEditorRobot/>
            <CircuitSelector/>
            <CanvasBirdEye/>
            <GazeboViewer/>
            <VncConsoleViewer/>
            </CircuitSelectorProvider>
        </Box>
    )
}

export default FollowLineReact;
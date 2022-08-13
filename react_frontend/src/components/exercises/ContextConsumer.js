import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/ExerciseContext";
import PropTypes from "prop-types";

function FollowLineReact(children) {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>{children}</ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

ExerciseProvider.propTypes = {
  children: PropTypes.node,
};

export default FollowLineReact;

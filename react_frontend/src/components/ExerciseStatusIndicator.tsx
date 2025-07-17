import React from "react";
import { Box, Typography } from "@mui/material";
import { styled, keyframes } from "@mui/material/styles";

// Define los posibles valores del estado como un tipo TypeScript
type StatusType = "ACTIVE" | "INACTIVE" | "PROTOTYPE";

interface StatusProps {
  status: StatusType;
}

const blink = keyframes`
  from { opacity: 0; }
  to { opacity: 1; }
`;

// Estilo para indicador con animación
const BlinkingIndicator = styled("div")<StatusProps>(({ status }) => ({
  backgroundColor:
    status === "ACTIVE" ? "green" : status === "INACTIVE" ? "red" : "orange",
  width: 10,
  height: 10,
  borderRadius: "50%",
  borderColor: "black",
  animation: `${blink} 3s linear infinite`,
}));

// Estilo para indicador sin animación
const Indicator = styled("div")<StatusProps>(({ status }) => ({
  backgroundColor:
    status === "ACTIVE" ? "green" : status === "INACTIVE" ? "red" : "orange",
  width: 10,
  height: 10,
  borderRadius: "50%",
  borderColor: "black",
}));

const ExerciseStatusIndicator: React.FC<StatusProps> = ({ status }) => {
  return (
    <Box
      sx={{
        display: "inline-flex",
        flexDirection: "row",
        alignItems: "center",
        justifyContent: "center",
        border: 1,
        borderColor: "white",
        py: 0.5,
        pl: 1,
        pr: 2,
        borderRadius: "16px",
      }}
    >
      <Typography
        color="white"
        variant="subtitle2"
        component="div"
        sx={{ marginRight: 2 }}
      >
        Status
      </Typography>
      <Indicator status={status} />
    </Box>
  );
};

export default ExerciseStatusIndicator;

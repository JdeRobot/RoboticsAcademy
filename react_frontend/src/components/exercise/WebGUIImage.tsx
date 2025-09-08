import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import VideocamOffOutlinedIcon from "@mui/icons-material/VideocamOffOutlined";
import { StyledImageContainer } from "Styles/exercise/WebGUIImage.styles";
import React from "react";

// camera
const WebGUIImage = ({
  style,
  id,
  src,
  errorMsg,
}: {
  style?: object;
  id?: string;
  src?: string;
  errorMsg?: string;
}) => {
  const theme = useAcademyTheme();

  return (
    <>
      <StyledImageContainer style={style} color={theme.palette.error}>
        {src ? (
          <img id={id} src={src} />
        ) : (
          <>
            <VideocamOffOutlinedIcon htmlColor={theme.palette.error} />
            <h3>{errorMsg ? errorMsg : "No image available"}</h3>
          </>
        )}
      </StyledImageContainer>
    </>
  );
};

export default WebGUIImage;

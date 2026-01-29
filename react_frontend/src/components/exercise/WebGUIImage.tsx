import React, { RefObject } from "react";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import VideocamOffOutlinedIcon from "@mui/icons-material/VideocamOffOutlined";
import { StyledImageContainer } from "Styles/exercise/WebGUIImage.styles";

// camera
const WebGUIImage = ({
  style,
  id,
  reference,
  src,
  errorMsg,
}: {
  style?: object;
  id?: string;
  reference?: RefObject<HTMLImageElement>;
  src?: string;
  errorMsg?: string;
}) => {
  const theme = useAcademyTheme();

  return (
    <>
      <StyledImageContainer style={style} color={theme.palette.error}>
        {src ? (
          <img ref={reference} id={id} src={src} />
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

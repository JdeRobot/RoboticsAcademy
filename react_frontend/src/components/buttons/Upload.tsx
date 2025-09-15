import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import FileUploadRoundedIcon from "@mui/icons-material/FileUploadRounded";
import { useRef } from "react";
import { publish } from "Helpers/utils";
import React from "react";

const UploadButton = () => {
  const theme = useAcademyTheme();
  const inputRef = useRef<HTMLInputElement>(null);

  const loadFile = (event: React.ChangeEvent<HTMLInputElement>) => {
    event.preventDefault();
    const fr = new FileReader();
    fr.onload = () => {
      if (fr.result) {
        publish("uploadOnlyCode", { code: fr.result as string });
      }
    };
    const files = event.target.files;

    if (files && files.length > 0) {
      fr.readAsText(files[0]);
    }
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      onClick={() => {
        if (inputRef.current) {
          inputRef.current.click();
        }
      }}
      id="upload-code"
      title="Upload code"
    >
      <FileUploadRoundedIcon htmlColor={theme.palette.text} />
      <input
        ref={inputRef}
        hidden
        accept=".py"
        type="file"
        onChange={loadFile}
      />
    </StyledHeaderButton>
  );
};

export default UploadButton;

import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import FileUploadRoundedIcon from "@mui/icons-material/FileUploadRounded";
import { useRef } from "react";
import { publish } from "Helpers/utils";

const UploadButton = () => {
  const theme = useAcademyTheme();
  const inputRef = useRef(null);

  const loadFile = (event: React.ChangeEvent<HTMLInputElement>) => {
    event.preventDefault();
    const fr = new FileReader();
    fr.onload = () => {
      if (fr.result) {
        publish("uploadOnlyCode", { code: fr.result as string });
      }
    };
    fr.readAsText(event.target.files?.[0]!);
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      onClick={() => {
        (inputRef.current as any).click();
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

import { StyledHeaderButton } from "Components/HeaderMenu/HeaderMenu.styles";
import { useTheme } from "jderobot-ide-interface";
import FileUploadRoundedIcon from "@mui/icons-material/FileUploadRounded";
import { useRef } from "react";

const UploadButton = ({
  loadFile,
}: {
  loadFile: (event: React.ChangeEvent<HTMLInputElement>) => void;
}) => {
  const theme = useTheme();
  const inputRef = useRef(null);

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

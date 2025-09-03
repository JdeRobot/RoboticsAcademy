import { StyledHeaderButton } from "Components/HeaderMenu/HeaderMenu.styles";
import { useTheme } from "jderobot-ide-interface";
import FileDownloadRoundedIcon from "@mui/icons-material/FileDownloadRounded";

const DownloadButton = ({ saveFile }: { saveFile: () => void }) => {
  const theme = useTheme();

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="download-code"
      onClick={saveFile}
      title="Download code"
    >
      <FileDownloadRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default DownloadButton;

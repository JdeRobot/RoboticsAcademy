import { StyledHeaderButton } from "Components/HeaderMenu/HeaderMenu.styles";
import { useTheme } from "jderobot-ide-interface";
import { useRef } from "react";
import PsychologyRoundedIcon from '@mui/icons-material/PsychologyRounded';

const DeepLearningButton = ({
  loadFile,
}: {
  loadFile: (event: React.ChangeEvent<HTMLInputElement>) => void;
}) => {
  const theme = useTheme();
  const inputRef = useRef(null);

  //TODO: add functionality

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      onClick={() => {
        (inputRef.current as any).click();
      }}
      id="upload-dl-model"
      title="Upload Deep Learning Model"
    >
      <PsychologyRoundedIcon htmlColor={theme.palette.text} />
      <input
        ref={inputRef}
        hidden
        accept=".onnx"
        type="file"
        onChange={loadFile}
      />
    </StyledHeaderButton>
  );
};

export default DeepLearningButton;

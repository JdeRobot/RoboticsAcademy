import { StyledHeaderButton } from "Components/headers/HeaderMenu.styles";
import { useRef, useState } from "react";
import PsychologyRoundedIcon from '@mui/icons-material/PsychologyRounded';
import { useAcademyTheme } from "Contexts/AcademyThemeContext";

const DeepLearningButton = ({
  setModel,
}: {
  setModel: Function;
}) => {
  const theme = useAcademyTheme();
  const inputRef = useRef(null);

  const [fileName, setFileName] = useState<string|undefined>(undefined);

  const handleUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
      const file = event.target.files?.[0];
      if (!file || !file.name.endsWith(".onnx")) {
        alert("Please upload a valid .onnx file");
        return;
      }

      const buffer = await file.arrayBuffer(); // Binary buffer

      console.log("====================================");
      console.log("File uploaded:", file.name);
      console.log("File size:", (file.size / (1024 * 1024)).toFixed(2), "MB");
      console.log("File type:", file.type);
      console.log("====================================");

      // Set the model in the DeepLearningModel component
      setModel(buffer);
      // Update the state with file details
      setFileName(file.name);
    };


  return (
    <StyledHeaderButton
      bgColor={fileName ? theme.palette.button.success : theme.palette.button.error}
      hoverColor={fileName ? theme.palette.button.hoverSuccess : theme.palette.button.hoverError}
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
        onChange={handleUpload}
      />
    </StyledHeaderButton>
  );
};

export default DeepLearningButton;

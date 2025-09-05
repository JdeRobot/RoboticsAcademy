import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import FileDownloadRoundedIcon from "@mui/icons-material/FileDownloadRounded";
import { publish, saveCode, subscribe, unsubscribe } from "Helpers/utils";
import { useEffect, useRef, useState } from "react";
import { useExercise } from "Contexts/ExerciseContext";

const DownloadButton = () => {
  const theme = useAcademyTheme();
  const exerciseContext = useExercise();
  const codeRef = useRef("");
  const isCodeUpdatedRef = useRef<boolean | undefined>(undefined);
  const [isCodeUpdated, _updateCode] = useState<boolean | undefined>(false);

  const updateCode = (data?: boolean) => {
    isCodeUpdatedRef.current = data;
    _updateCode(data);
  };

  useEffect(() => {
    subscribe("autoSaveCompleted", () => {
      updateCode(true);
    });

    return () => {
      unsubscribe("autoSaveCompleted", () => {});
    };
  }, []);

  useEffect(() => {
    codeRef.current = exerciseContext.code;
  }, [exerciseContext]);

  const saveFile = (save?: boolean) => {
    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      console.log("Try autosave", isCodeUpdated);
      return setTimeout(saveFile, 100, true);
    }

    saveCode("academy", codeRef.current);
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="download-code"
      onClick={() => saveFile(undefined)}
      title="Download code"
    >
      <FileDownloadRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default DownloadButton;

import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import FileDownloadRoundedIcon from "@mui/icons-material/FileDownloadRounded";
import { publish, subscribe, unsubscribe, zipCodeFiles } from "Helpers/utils";
import { useEffect, useRef, useState } from "react";
import React from "react";
import JSZip from "jszip";
import { getFileList } from "Api";

const DownloadButton = ({ project }: { project: string }) => {
  const theme = useAcademyTheme();
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

  const saveFile = async (save?: boolean) => {
    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      console.log("Try autosave", isCodeUpdated);
      return setTimeout(saveFile, 100, true);
    }

    const zip = new JSZip();
    const files = await getFileList(project);
    await zipCodeFiles(zip, JSON.parse(files), project);

    zip.generateAsync({ type: "blob" }).then(function (content) {
      // Create a download link and trigger download
      const url = window.URL.createObjectURL(content);
      const a = document.createElement("a");
      a.style.display = "none";
      a.href = url;
      a.download = `${project}.zip`; // Set the downloaded file's name
      document.body.appendChild(a);
      a.click();
      window.URL.revokeObjectURL(url); // Clean up after the download
    });
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

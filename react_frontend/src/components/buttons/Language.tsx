import React from "react";
import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { CppIcon, PythonIcon } from "Icons/index";
import SyncRoundedIcon from "@mui/icons-material/SyncRounded";

const LanguageButton = ({
  language,
  setter,
  loading,
}: {
  language: string;
  setter: (value: string) => void;
  loading?: boolean;
}) => {
  const theme = useAcademyTheme();

  console.log(loading)

  const switchLanguage = () => {
    setter(language === "python" ? "cpp" : "python");
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="language-button"
      onClick={switchLanguage}
      title="Switch Language"
      disabled={loading}
    >
      {loading ? (
        <SyncRoundedIcon htmlColor={theme.palette.text} id="loading-spin" />
      ) : (
        <>
          {language === "cpp" ? (
            <CppIcon fill={theme.palette.text} />
          ) : (
            <PythonIcon fill={theme.palette.text} />
          )}
        </>
      )}
    </StyledHeaderButton>
  );
};

export default LanguageButton;

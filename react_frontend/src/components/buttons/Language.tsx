import React from "react";
import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { CppIcon, PythonIcon } from "Icons/index";

const LanguageButton = ({
  language,
  setter,
}: {
  language: string;
  setter: (value: string) => void;
}) => {
  const theme = useAcademyTheme();

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
    >
      {language === "cpp" ? (
        <CppIcon fill={theme.palette.text} />
      ) : (
        <PythonIcon fill={theme.palette.text} />
      )}
    </StyledHeaderButton>
  );
};

export default LanguageButton;

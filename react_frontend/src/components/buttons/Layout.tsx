import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import SpaceDashboardRoundedIcon from "@mui/icons-material/SpaceDashboardRounded";
import React from "react";
import { Layout } from "jderobot-ide-interface";
import { Dropdown } from "./Dropdown";

const LayoutButton = ({
  setLayout,
}: {
  setLayout: (layout: Layout) => void;
}) => {
  const theme = useAcademyTheme();

  return (
    <Dropdown
      id="layout"
      title="Layout"
      width={120}
      setter={setLayout}
      possibleValues={["only-editor", "only-viewers", "both"]}
    >
      <SpaceDashboardRoundedIcon htmlColor={theme.palette.text} />
    </Dropdown>
  );
};

export default LayoutButton;

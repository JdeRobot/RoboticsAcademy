import BuildRoundedIcon from '@mui/icons-material/BuildRounded';
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { type ViewersEntry } from "jderobot-ide-interface";
import { Dropdown } from './Dropdown';
import { publish } from 'Helpers/utils';
import React from "react";

const TerminateUniverseButton = ({ tools }: { tools: ViewersEntry[] }) => {
  const theme = useAcademyTheme();

  const selectTools = (newTool: string) => {
    publish("changeToolGroup", { tool: newTool });
  };

  let toggleGroup = undefined;
  const groups: string[] = [];

  for (let index = 0; index < tools.length; index++) {
    const element = tools[index];
    if (element.group !== undefined) {
      if (groups.includes(element.group)) {
        toggleGroup = element.group;
      } else {
        groups.push(element.group);
      }
    }
  }

  const toggles = [];

  if (toggleGroup !== undefined) {
    for (let index = 0; index < tools.length; index++) {
      const element = tools[index];
      if (element.group === toggleGroup) {
        toggles.push(element.name);
      }
    }
  }

  if (toggles.length === 0) {
    return <></>
  }

  return (
    <Dropdown
      id="tool"
      title="Switch Tool"
      width={300}
      setter={selectTools}
      possibleValues={toggles}
    >
      <BuildRoundedIcon htmlColor={theme.palette.text} />
    </Dropdown>
  );
};

export default TerminateUniverseButton;

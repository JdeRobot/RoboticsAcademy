import {
  StyledDropdown,
  StyledHeaderButton,
} from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import SpaceDashboardRoundedIcon from "@mui/icons-material/SpaceDashboardRounded";
import { ReactNode, useRef, useState } from "react";
import React from "react";
import { Layout } from "jderobot-ide-interface";

const LayoutButton = ({
  setLayout,
}: {
  setLayout: (layout: Layout) => void;
}) => {
  const theme = useAcademyTheme();

  return (
    <Dropdown
      id="open-settings-manager"
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

const Dropdown = ({
  id,
  title,
  width,
  setter,
  possibleValues,
  children,
}: {
  id: string;
  title: string;
  width: number;
  setter: (layout: Layout) => void;
  possibleValues: Layout[];
  children: ReactNode;
}) => {
  const [open, setOpen] = useState<boolean>(false);
  const [right, setRight] = useState<number>(width / 2 + 13);
  const theme = useAcademyTheme();
  const dropdown = useRef<HTMLDivElement>(null);

  const changeValue = (e: React.MouseEvent<HTMLElement>, value: Layout) => {
    e.preventDefault();
    setter(value);
    setOpen(false);
  };

  const closeOpenMenus = (e: MouseEvent) => {
    if (open && !dropdown.current?.contains(e.target as Node)) {
      setOpen(false);
    }
  };

  const checkPosition = (x: number) => {
    if (x + width / 2 > window.innerWidth) {
      // To the left
      setRight(x);
    } else if (x < width / 2) {
      // To the right
      setRight(x - width);
    } else {
      // In the middle
      setRight(x - width / 2 + 13);
    }
  };

  document.addEventListener("mousedown", closeOpenMenus);

  return (
    <div ref={dropdown}>
      <StyledHeaderButton
        bgColor={theme.palette.primary}
        hoverColor={theme.palette.secondary}
        roundness={theme.roundness}
        id={id}
        title={title}
        onClick={(e) => {
          checkPosition(e.clientX);
          e.preventDefault();
          setOpen(!open);
        }}
      >
        {children}
      </StyledHeaderButton>
      {open && (
        <StyledDropdown
          color={theme.palette.text}
          bgColor={theme.palette.primary}
          hoverColor={theme.palette.secondary}
          roundness={theme.roundness}
          style={{ width: `${width}px`, left: `${right}px` }}
        >
          {possibleValues.map((name) => (
            <button
              key={`layout-${name}`}
              onClick={(e: React.MouseEvent<HTMLElement>) => changeValue(e, name)}
            >
              {name}
            </button>
          ))}
        </StyledDropdown>
      )}
    </div>
  );
};

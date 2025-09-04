import styled from "styled-components";

const primaryColor = "#666";

interface StyledHeaderTextProps {
  color?: string;
}

export const StyledHeaderText = styled.h1<StyledHeaderTextProps>`
  font-size: 25px;
  margin: 0;
  color: ${(p) => p.color ?? primaryColor};
`;

interface StyledProjectProps {
  color?: string;
}

export const StyledProject = styled.span<StyledProjectProps>`
  display: flex;
  flex-direction: column;
  background-color: none;
  color: ${(p) => p.color ?? primaryColor};
  padding: 10px;
  justify-content: center;
  margin-left: auto;

  & div {
    font-weight: bold;
  }
`;

export const StyledHeaderButtonContainer = styled.div`
  display: flex;
  justify-content: flex-end;
  align-items: center;
  margin-left: auto;
`;

interface StyledHeaderButtonProps {
  bgColor?: string;
  hoverColor?: string;
  roundness?: number;
}

export const StyledHeaderButton = styled.button<StyledHeaderButtonProps>`
  display: flex;
  justify-content: center;
  width: 32px;
  height: 32px;
  background-color: ${(p) => p.bgColor ?? primaryColor};
  border: 0;
  margin-left: 6px;
  margin-right: 6px;
  padding: 0 0 0 0;
  align-content: center;
  flex-wrap: wrap;
  border-radius: ${(p) => p.roundness ?? 1}px;

  &:hover {
    background-color: ${(p) => p.hoverColor ?? primaryColor};
  }

  &:focus {
    outline: none;
  }

  & svg {
    width: 24px;
    height: 24px;
  }
`;

interface StyledDropdownProps {
  color?: string;
  bgColor?: string;
  hoverColor?: string;
  roundness?: number;
}

export const StyledDropdown = styled.div<StyledDropdownProps>`
  border-radius: ${(p) => p.roundness ?? 1}px;
  display: flex;
  position: absolute;
  z-index: 10000;
  flex-direction: column;
  background-color: ${(p) => p.bgColor ?? primaryColor};
  border: 1px black solid;
  margin-top: 10px;

  & button {
    border-radius: 0 !important;
    border-top-left-radius: 0px;
    border-top-right-radius: 0px;
    margin-bottom: 0 !important;
    opacity: 1 !important;
    background-color: ${(p) => p.bgColor ?? primaryColor};
    color: ${(p) => p.color ?? primaryColor};
    border: 0;
    padding: 5px 5px 5px 5px;
    font-size: 16px;

    &:first-of-type {
      border-top-left-radius: ${(p) => p.roundness ?? 1}px !important;
      border-top-right-radius: ${(p) => p.roundness ?? 1}px !important;
    }

    &:last-of-type {
      border-bottom-left-radius: ${(p) => p.roundness ?? 1}px !important;
      border-bottom-right-radius: ${(p) => p.roundness ?? 1}px !important;
    }

    &:hover {
      background-color: ${(p) => p.hoverColor ?? primaryColor};
    }

    &:focus {
      outline: none;
    }
  }
`;

import styled from "styled-components";

interface StyledTitleProps {
  bg: string;
  scroll: string;
  color: string;
  roundness?: number;
}

export const Terminal = styled.div<StyledTitleProps>`
  width: 100%;
  height: 100%;
  display: flex;

  background-color: ${(p) => p.bg};

  .xterm {
    padding: 8px;
    flex-grow: 1;
  }

  .xterm-viewport {
    background-color: unset !important;
  }

  .xterm-scrollable-element {
    background-color: unset !important;
    height: 100%;
    width: 100%;
  }

  .vertical {
    .slider {
      left: 6px !important;
      width: 6px !important;
      border-radius: 3px;
      background: ${(p) => p.scroll} !important;
      opacity: 0.3;
      &:hover {
        opacity: 0.7;
      }
    }
  }

  span {
    color: ${(p) => p.color} !important;
  }

  border-radius: ${(p) => p.roundness ?? 1}px 0 0 ${(p) => p.roundness ?? 1}px;
  overflow: hidden;
  flex-grow: 1;
`;

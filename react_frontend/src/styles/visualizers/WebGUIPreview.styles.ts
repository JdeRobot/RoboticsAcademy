import styled from "styled-components";

const primaryColor = "#666";

interface StyledWebGUIErrorProps {
  color?: string;
}

export const StyledWebGUIError = styled.div<StyledWebGUIErrorProps>`
  position: absolute;
  display: flex;
  flex-direction: column;
  gap: 10px;
  justify-content: center;
  align-items: center;

  & svg {
    height: 120px;
    width: 120px;
  }

  & h3 {
    color: ${(p) => p.color ?? primaryColor};
  }
`;

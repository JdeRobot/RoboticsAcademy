import styled from "styled-components";

const primaryColor = "#666";

interface StyledStubBackgroundProps {
  bgColor?: string;
}

export const StyledStubBackground = styled.div<StyledStubBackgroundProps>`
  height: 100vh;
  position: absolute;
  overflow: hidden;
  top: 0px;
  left: 0px;
  width: 100%;
  z-index: -1;
  background-color: ${(p) => p.bgColor ?? primaryColor};
`;

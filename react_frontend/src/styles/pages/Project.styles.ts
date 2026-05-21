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

export const StyledErrorContainer = styled.div<StyledStubBackgroundProps>`
  padding: 28px;
  background-color: ${(p) => p.bgColor ?? primaryColor};
  overflow: auto;
  height: calc(100vh - 50px);
  display: flex;
  align-items: center;
  justify-content: center;
  flex-direction: column;
`;

interface StyledErrorMessageProps {
  color?: string;
}

export const StyledErrorMessage = styled.h1<StyledErrorMessageProps>`
  color: ${(p) => p.color ?? primaryColor};
  text-align: center;
`;

import styled from "styled-components";

const primaryColor = "#666";

interface StyledVideoInputContainerProps {
  color?: string;
}

export const StyledVideoInputContainer = styled.div<StyledVideoInputContainerProps>`
  padding: 64px;
  width: 100%;
  height: 100%;
  text-align: center;
  cursor: pointer;
  background-color: unset;
  transition: all 0.3s ease;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  position: relative;
  overflow: hidden;
  z-index: 0;

  &:hover {
    box-shadow: inset 20px 0 40px ${(p) => p.color ?? primaryColor},
      inset -20px 0 40px ${(p) => p.color ?? primaryColor};
    & .upload-bg: {
      opacity: 5%;
    }
  }
`;

export const StyledVideoContainer = styled.div`
  position: relative;
  overflow: hidden;
  width: 100%;
  height: 100%;
  background-color: unset;
  z-index: 10;

  &:hover .controls {
    opacity: 1;
  }
`;

export const StyledVideo = styled.video`
  position: absolute;
  left: 0;
  top: 0;
  width: 100%;
  height: 100%;
  objectfit: contain;
  display: block;
`;

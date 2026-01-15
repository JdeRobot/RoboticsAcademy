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

interface StyledCameraErrorProps {
  color?: string;
}

export const StyledCameraError = styled.div<StyledCameraErrorProps>`
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

interface StyledWebCamVideoProps {
  visible: boolean;
}

const handleVisibility = (p: StyledWebCamVideoProps) => {
  if (!p.visible) {
    return `display: none;`;
  }
};

export const StyledWebCamVideo = styled.video<StyledWebCamVideoProps>`
  position: absolute;
  left: 0;
  top: 0;
  width: 100%;
  height: 100%;
  objectFit: contain;
  display: block;
  ${handleVisibility}
`;

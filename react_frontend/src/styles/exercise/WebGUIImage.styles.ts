import styled from "styled-components";

const primaryColor = "#666";

interface StyledImageContainerProps {
  color?: string;
}

export const StyledImageContainer = styled.div<StyledImageContainerProps>`
  position: absolute;
  display: flex;
  flex-direction: column;
  gap: 10px;
  justify-content: center;
  align-items: center;
  width: 50%;
  height: 100%;

  & img {
    width: 100%;
    height: 100%;
  }

  & svg {
    height: 120px;
    width: 120px;
  }

  & h3 {
    color: ${(p) => p.color ?? primaryColor};
  }
`;

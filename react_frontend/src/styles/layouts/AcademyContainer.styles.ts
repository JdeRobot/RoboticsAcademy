import styled from "styled-components";

const primaryColor = "#666";

interface StyledAcademyContainerProps {
  bgColor?: string;
}

export const StyledAcademyContainer = styled.div<StyledAcademyContainerProps>`
  padding: 28px;
  background-color: ${(p) => p.bgColor ?? primaryColor};
  overflow: auto;
  height: calc(100vh - 50px);
`;

export const StyledExerciseList = styled.div`
  margin: 0px auto;
  padding: 8px;
  display: flex;
  flex-flow: wrap;
  place-content: center;
  align-items: center;
  gap: 2rem;
`;

export const StyledAcademyLoadingMsg = styled.div`
  max-width: 1200px;
  min-height: calc(100vh - 106px);
  margin: 0 auto;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  font-family: "Quicksand", sans-serif;
  font-size: 5rem;
  font-weight: 800;
  background-image: linear-gradient(
    -225deg,
    #231557 0%,
    #44107a 29%,
    #ff1361 67%,
    #fff800 100%
  );
  background-size: auto auto;
  background-clip: border-box;
  background-size: 200% auto;
  color: #fff;
  background-clip: text;
  text-fill-color: transparent;
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  animation: textclip 2s linear infinite;
`;

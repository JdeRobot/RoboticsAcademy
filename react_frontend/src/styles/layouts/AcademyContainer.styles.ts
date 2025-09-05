import styled from "styled-components";

const primaryColor = "#666";

interface StyledAcademyContainerProps {
  bgColor?: string;
}

export const StyledAcademyContainer = styled.div<StyledAcademyContainerProps>`
  padding-top: 28px;
  padding-bottom: 28px;
  background-color:${(p) => p.bgColor ?? primaryColor};
`;

export const StyledExerciseList = styled.div`
  max-width: 1200px;
  min-height: calc(100vh - 106px);
  margin: 0 auto;
  display: grid;
  grid-gap: 1rem;
  padding: 8px;
  grid-auto-rows: 1fr;

  @media (min-width: 600px) {
    grid-template-columns: repeat(3, 1fr);
  }

  @media (min-width: 900px) {
    grid-template-columns: repeat(4, 1fr);
  }
`;

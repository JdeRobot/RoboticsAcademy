import { ExerciseStatus } from "src/types/exercises";
import styled from "styled-components";

const primaryColor = "#666";

interface StyledExerciseCardContainerProps {
  state: ExerciseStatus;
}

const handleStatus = (p: StyledExerciseCardContainerProps) => {
  switch (p.state) {
    case "ACTIVE":
      return `border-color: green;`
    case "PROTOTYPE":
      return `border-color: orange;`
    default:
      return `border-color: red;`
  }
};

export const StyledExerciseCardContainer = styled.div<StyledExerciseCardContainerProps>`
  display: flex;
  flex-direction: column;
  justify-content: spacebetween;
  background-color: black;
  border-radius: 20px;
  border: 5px solid;
  position: relative;
  overflow: hidden;
  width: 100%;
  aspect-ratio: 9/10;
  ${handleStatus}
`;

export const StyledExerciseCardArea = styled.div`
  flex-grow: 1;
  display: flex;
  flex-direction: column;
  align-items: flex-start;
`;

export const StyledExerciseCardInfoContainer = styled.div`
    width: 100%;
    height: 15%;
    position: absolute;
    background: rgba(0, 0, 0, 0.68);
    animation: position;
    transition: height 0.8s ease;
    bottom: 0;
    padding: 8px;
    display: inline-flex;
    flex-direction: column;
    flex-grow: 1;

    & #exercise-info,
    .exercise-tag-list {
        opacity: 0%;
        transition: opacity 1s ease;
    }

    &:hover {
        height: 100%;

        & #exercise-info,
        .exercise-tag-list {
            opacity: 100%;
        }
    }
`;

export const StyledExerciseCardTagList = styled.div`
    padding: 4px;
`;
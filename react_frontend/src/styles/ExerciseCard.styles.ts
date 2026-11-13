import { ExerciseStatus } from "Types/exercises";
import { Link } from "react-router";
import styled from "styled-components";

interface StyledExerciseCardContainerProps {
  state: ExerciseStatus;
}

const handleStatus = (p: StyledExerciseCardContainerProps) => {
  switch (p.state) {
    case "ACTIVE":
      return `green`;
    case "PROTOTYPE":
      return `orange`;
    default:
      return `red`;
  }
};

export const StyledExerciseCardContainer = styled(
  Link,
)<StyledExerciseCardContainerProps>`
  display: flex;
  flex-direction: column;
  justify-content: spacebetween;
  background-color: black;
  border-radius: 20px;
  border: 5px solid;
  position: relative;
  overflow: hidden;
  width: 20rem;
  aspect-ratio: 9/10;
  outline: none;
  border-color: ${handleStatus};

  &:hover {
    box-shadow: 0 0 30px ${handleStatus};
  }

  &:focus-visible {
    box-shadow: 0 0 30px ${handleStatus};
  }
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
  color: #fff;

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

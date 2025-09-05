import React, { useEffect, useState, useContext } from "react";
import { ExerciseCard } from "./../ExerciseCard";
import { useHomepage } from "Contexts/HomepageContext";
import { Exercise, Filters } from "src/types/exercises";
import { listExercises } from "Helpers/api";
import {
  StyledAcademyContainer,
  StyledExerciseList,
} from "Styles/layouts/AcademyContainer.styles";
import { useTheme } from "jderobot-ide-interface";

const AcademyContainer = () => {
  const { getSearchBarText, getFilterItemsList } = useHomepage();
  const theme = useTheme();
  const [loading, setLoading] = useState<boolean>(true);
  const [exerciseList, setExerciseList] = useState<Exercise[] | undefined>(
    undefined
  );
  const filterText: string = getSearchBarText();

  const getExercises = async () => {
    const exercises = await listExercises();
    setExerciseList(exercises);
    setLoading(false);
  };

  useEffect(() => {
    getExercises();
  }, []);

  if (loading) {
    return <div className="loading-list-message">Loading exercises</div>;
  }

  const filteredData: Exercise[] =
    exerciseList?.filter((el: Exercise) => {
      if (filterText === "") {
        return el;
      }

      const filterItemsList: Filters[] = getFilterItemsList();
      for (let i in filterItemsList) {
        const value = el[filterItemsList[i]];
        if (value.toLowerCase().includes(filterText.toLowerCase())) {
          return true;
        }
      }
      return false;
    }) || [];

  return (
    <StyledAcademyContainer bgColor={theme.palette.background}>
      <StyledExerciseList>
        {filteredData.map((exercise) => (
          <ExerciseCard
            id={exercise.exercise_id}
            name={exercise.name}
            description={exercise.description}
            tags={exercise.tags}
            status={exercise.status}
          />
        ))}
      </StyledExerciseList>
    </StyledAcademyContainer>
  );
};

export default AcademyContainer;

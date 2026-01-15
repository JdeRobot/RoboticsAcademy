import React, { useEffect, useState } from "react";
import { ExerciseCard } from "./../ExerciseCard";
import { useHomepage } from "Contexts/HomepageContext";
import { Exercise, Filters } from "src/types/exercises";
import {
  StyledAcademyContainer,
  StyledAcademyLoadingMsg,
  StyledExerciseList,
} from "Styles/layouts/AcademyContainer.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { getExerciseList } from "Api";

const AcademyContainer = () => {
  const { getSearchBarText, getFilterItemsList } = useHomepage();
  const theme = useAcademyTheme();
  const [loading, setLoading] = useState<boolean>(true);
  const [exerciseList, setExerciseList] = useState<Exercise[] | undefined>(
    undefined
  );
  const filterText: string = getSearchBarText();

  const getExercises = async () => {
    const exercises = await getExerciseList();
    setExerciseList(exercises);
    setLoading(false);
  };

  useEffect(() => {
    getExercises();
  }, []);

  if (loading) {
    return (
      <StyledAcademyContainer bgColor={theme.palette.bg}>
        <StyledAcademyLoadingMsg>Loading exercises</StyledAcademyLoadingMsg>
      </StyledAcademyContainer>
    );
  }

  const filteredData: Exercise[] =
    exerciseList?.filter((el: Exercise) => {
      if (filterText === "") {
        return el;
      }

      const filterItemsList: Filters[] = getFilterItemsList();
      for (const i in filterItemsList) {
        const value = el[filterItemsList[i]];
        if (value.toLowerCase().includes(filterText.toLowerCase())) {
          return true;
        }
      }
      return false;
    }) || [];

  return (
    <StyledAcademyContainer bgColor={theme.palette.bg}>
      <StyledExerciseList>
        {filteredData.map((exercise) => (
          <ExerciseCard
            key={exercise.exercise_id}
            exercise_id={exercise.exercise_id}
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

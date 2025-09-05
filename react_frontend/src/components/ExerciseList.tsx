import React, { useEffect, useState, useContext } from "react";
import "../styles/ExerciseList.css";
import { ExerciseCard } from "./ExerciseCard";
import { useHomepage } from "../contexts/HomepageContext";

const serverBase: string = `${document.location.protocol}//${document.location.hostname}:7164`;

interface Exercise {
  exercise_id: string;
  name: string;
  description: string;
  tags: string;
  status: "ACTIVE" | "INACTIVE" | "PROTOTYPE";
}

const ExerciseList: React.FC = () => {
  const { getSearchBarText, getFilterItemsList } = useHomepage();
  const [loading, setLoading] = useState<boolean>(true);
  const [exerciseList, setExerciseList] = useState<Exercise[] | undefined>(undefined);
  const filterText: string = getSearchBarText();

  useEffect(() => {
    const apiURL: string = `${serverBase}/api/v1/exercises/`;
    fetch(apiURL)
      .then((res) => res.json())
      .then((exercises: Exercise[]) => {
        setExerciseList(exercises);
        setLoading(false);
      });
  }, [setExerciseList]);

  if (loading) {
    return <div className="loading-list-message">Loading exercises</div>;
  }

  const filteredData: Exercise[] = exerciseList?.filter((el: any ) => {
    if (filterText === "") {
      return el;
    } else {
      const filterItemsList: string[] = getFilterItemsList();
      for (let i in filterItemsList) {
        const filterItem: string = filterItemsList[i];
        const value= el[filterItem];
        if (typeof value === "string" && value.toLowerCase().includes(filterText.toLowerCase())) {
            return true;
        }
      }
    }
    return false;
  }) || [];

  return (
    <div className={"exercise-list"}>
      {filteredData.map((exercise) => (
        <ExerciseCard
          id={exercise.exercise_id}
          name={exercise.name}
          description={exercise.description}
          tags={exercise.tags}
          status={exercise.status}
        />
      ))}
    </div>
  );
};

export default ExerciseList;

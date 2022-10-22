import { useEffect, useState, useContext } from "react";
import "../styles/ExerciseList.css";
import { ExerciseCardV2 } from "./ExerciseCard";
import React from "react";
import HomepageContext from "../contexts/HomepageContext";

const serverBase = `${document.location.protocol}//${document.location.hostname}:8000`;

const ExerciseList = () => {
  const { getSearchBarText } = useContext(HomepageContext);
  // const [listState, setListState] = useState({
  //   loading: true,
  //   exercises: null,
  // });
  const [loading, setLoading] = useState(true);
  const [exerciseList, setExerciseList] = useState();
  const filterText = getSearchBarText();

  useEffect(() => {
    // setListState({ loading: true, exercises: null });
    const apiURL = `${serverBase}/api/v1/exercises/`;
    fetch(apiURL)
      .then((res) => res.json())
      .then((exercises) => {
        setExerciseList(exercises);
        setLoading(false);
        // setListState({ loading: false, exercises: exercises });
      });
  }, [setExerciseList]);

  if (loading) {
    return <div className="loading-list-message">Loading exercises</div>;
  }

  const filteredData = exerciseList.filter((el) => {
    if (filterText === "") {
      return el;
    } else {
      return el.name.toLowerCase().includes(filterText);
    }
  });

  return (
    <div className={"exercise-list"}>
      {filteredData.map((exercise) => {
        return (
          <ExerciseCardV2
            key={exercise.exercise_id}
            exerciseid={exercise.exercise_id}
            name={exercise.name}
            description={exercise.description}
            tags={exercise.tags}
            status={exercise.status}
          />
        );
      })}
    </div>
  );
};

export default ExerciseList;

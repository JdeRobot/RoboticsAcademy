import { useEffect, useState, useContext } from "react";
import "../styles/ExerciseList.css";
import { ExerciseCardV2 } from "./ExerciseCard";
import React from "react";
import HomepageContext from "../contexts/HomepageContext";

const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;

let ros_version;

const ExerciseList = () => {
  const { getSearchBarText, getFilterItemsList } = useContext(HomepageContext);
  // const [listState, setListState] = useState({
  //   loading: true,
  //   exercises: null,
  // });
  const [loading, setLoading] = useState(true);
  const [exerciseList, setExerciseList] = useState();
  const filterText = getSearchBarText();

  const filterByVersion = (data) => {
    // Requests ROS version and filters exercises by ROS tag
    const rosVersionURL = `${serverBase}/exercises/ros_version/`;
    fetch(rosVersionURL)
      .then((res) => res.json())
      .then((msg) => {
        ros_version = msg.version;
        // If ROS is installed
        if (!isNaN(parseInt(ros_version))) {
          data = data.filter((exercise) =>
            exercise.tags.includes(`ROS${ros_version}`)
          );
          setExerciseList(data);
          setLoading(false);
        }
        // If ROS is not installed (local + RADI developer)
        else {
          setExerciseList(data);
          setLoading(false);
        }
      })
      .catch((error) => {
        setExerciseList(data);
        setLoading(false);
      });
  };

  useEffect(() => {
    // setListState({ loading: true, exercises: null });
    const apiURL = `${serverBase}/api/v1/exercises/`;
    fetch(apiURL)
      .then((res) => res.json())
      .then((exercises) => {
        filterByVersion(exercises);
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
      const filterItemsList = getFilterItemsList();
      for (var i in filterItemsList) {
        const filterItem = filterItemsList[i];
        if (el[filterItem].toLowerCase().includes(filterText) === true) {
          return true;
        }
      }
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

import {useEffect, useState} from "react";
import '../styles/ExerciseList.css'
import configuration from "../config.json";

const serverBase = `${document.location.protocol}//${document.location.hostname}:8000`;

const ExerciseList = (props) => {
  const [listState, setListState] = useState({
    loading: true,
    exercises: null
  })
  
  useEffect(() => {
    setListState({ loading: true, exercises: null });
    const apiURL = `${serverBase}/api/v1/exercises`;
    fetch(apiURL)
      .then((res) => res.json())
      .then((exercises) => {
        setListState({ loading: false, exercises: exercises })
      });
  }, [setListState])
  
  if(listState.loading) {
    return (
      <div className='loading-list-message'>
        Loading exercises
      </div>
    )
  }
  
  return (
    <div className={'exercise-list'}>
      {
        listState.exercises.map((exercise) => {
          return <ExerciseCard key={exercise.exercise_id}
                               exerciseid={exercise.exercise_id}
                               name={exercise.name}
                               description={exercise.description} />
              
        })
      }
    </div>
  )
}

const Teaser = (props) => {
  const teaser_type = props.type;
  const teaser_url = props.url;

  if(teaser_type==='video') {
    return (
      <video className="card-img" autoPlay muted loop>
        <source src={teaser_url} alt={props.name}/>
      </video>
    );
  } else if(teaser_type==='image') {
    return (
      <img src={teaser_url} className={"card-img"}/>
    );
  }
}

const ExerciseCard = (props) => {
  const exerciseURL = `${configuration.academy.exercises.exercise_url}`;
  const teaser = configuration.academy.exercises.teaser;

  const navigateToExercise = () => {
    const url = exerciseURL.interpolate(props);
    window.location.href = url;
  }
  
  return (
    <div className='exercise-card' card-id={props.exerciseid} onClick={ () => navigateToExercise() }>
      <Teaser type={configuration.academy.exercises.teaser.type}
              url={teaser.url.interpolate(props)} />
      <div className='name'>{props.name}</div>
      <div className='description'>{props.description}</div>
    </div>
  );
}

export {
  ExerciseList,
  ExerciseCard
}
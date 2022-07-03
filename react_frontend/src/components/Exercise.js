import React from 'react';
import '../styles/Exercise.css';
import AceEditor from "./exercises/AceEditor";
import ProminentAppBar from "./exercises/ProminentAppBar";
import RoboticsTheme from './RoboticsTheme';

function Exercise(props) {

  return (
    <RoboticsTheme>
      <div className="Exercise">
        <ProminentAppBar/>
        <div id={"react-content"}></div>
      </div>
    </RoboticsTheme>
  );
}

export default Exercise;
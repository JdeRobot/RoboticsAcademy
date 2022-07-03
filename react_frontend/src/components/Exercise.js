import React from 'react';
import '../styles/Exercise.css';
import AceEditorRobot from "./exercises/AceEditorRobot";
import ProminentAppBar from "./exercises/ProminentAppBar";
import RoboticsTheme from './RoboticsTheme';

function Exercise() {
  return (
    <RoboticsTheme>
      <div className="Exercise">
        <ProminentAppBar/>
        <AceEditorRobot/>
      </div>
    </RoboticsTheme>
  );
}

export default Exercise;
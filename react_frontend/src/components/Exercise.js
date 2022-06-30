import React from 'react';
import '../styles/Exercise.css';
import AceEditor from "./exercises/AceEditor";
import ProminentAppBar from "./exercises/ProminentAppBar";
import RoboticsTheme from './RoboticsTheme';

function Exercise() {
  return (
    <RoboticsTheme>
      <div className="Exercise">
        <ProminentAppBar/>
        <AceEditor/>
      </div>
    </RoboticsTheme>
  );
}

export default Exercise;
import React from 'react';
import '../styles/Exercise.css';
import RoboticsTheme from './RoboticsTheme';
import {CircuitSelectorProvider} from "../contexts/CircuitSelectorContext";

function Exercise() {

  return (

    <RoboticsTheme>
        <CircuitSelectorProvider>
        <div className="Exercise">
        </div>
        </CircuitSelectorProvider>
    </RoboticsTheme>
  );
}

export default Exercise;
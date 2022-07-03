import React from 'react';
import './exercise-index.css';
import Exercise from './components/Exercise';
import {createRoot} from 'react-dom/client';
import './libs/tools.js';

import AceEditor from './components/exercises/AceEditor';
import GazeboViewer from './components/exercises/GazeboViewer';
import ProminentAppBar from './components/exercises/ProminentAppBar';
import VncConsoleViewer from './components/exercises/VncConsoleViewer';

const container = document.getElementById('exercise');
const root = createRoot(container);

window.RoboticsExerciseComponents = (function() {
  const components = {
    AceEditor: AceEditor,
    GazeboViewer: GazeboViewer,
    ProminentAppBar: ProminentAppBar,
    VncConsoleViewer: VncConsoleViewer
  };

  const render = function (component, container, props, children, callback) {
    if(typeof component == 'string')
      component = components[component]

    const element = React.createElement(component, props, children, callback);
    const root = createRoot(container);
    root.render(element);
  };

  return {
    render: render,
    components: components
  }
})();

root.render(
    <React.StrictMode>
      <Exercise></Exercise>
    </React.StrictMode>
);

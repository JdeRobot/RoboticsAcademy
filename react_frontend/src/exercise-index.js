import React, {lazy} from 'react';
import convertHtmlToReact from '@hedgedoc/html-to-react';
import './exercise-index.css';
import Exercise from './components/Exercise';
import {createRoot} from 'react-dom/client';
import './libs/tools.js';
import {flushSync} from 'react-dom';
import CommsManager from './libs/comms_manager';

const container = document.getElementById('exercise');
const root = createRoot(container);

window.RoboticsExerciseComponents = (function() {
  const createElement = function(element) {
    const children = Array.from(element.childNodes).map((child) => createElement(child));
    return createElement(element, {}, children);
  }

  const renderComponent = function(component, container, props) {
    const innerHTML = container.innerHTML.trim();
    let react_children = null;
    if(innerHTML !== "") {
      react_children = convertHtmlToReact(innerHTML, {});
    }
    const element = React.createElement(component, props, react_children);
    const root = createRoot(container);
    flushSync(() => {
      root.render(element);
    })
  };

  const renderContext = function(component_name, container, props, children, callback) {
    const components = require.context('./components', true, /^.*\.(js|jsx)$/, 'sync');
    const component = components(`${component_name}.js`);
    renderComponent(component.default, container, props, children, callback);
  };

  const renderImport = async function (component_name, container, props, callback) {
    const path = component_name.split('/');
    const children = Array.from(container.childNodes);

    if(path[0]==='exercise') {
      await import(`exercise/${path[path.length-1]}.js`).then((component) => {
        renderComponent(component.default, container, props, children, callback);
      });
    } else {
      await import(`/${component_name}.js`).then((component) => {
        renderComponent(component.default, container, props, children, callback);
      });
    }
  };

  const render = async function(renderers) {
    for(let i=0, length=renderers.length; i < length; i++) {
      await renderImport(...renderers[i]);
    }
  }

  const ramHost = window.location.hostname;
  const ramPort = 7163;
  const ramManager = CommsManager(`ws://${ramHost}:${ramPort}`);

  return {
    render: render,
    commsManager: ramManager
  }
})();

root.render(
    <React.StrictMode>
      <Exercise></Exercise>
    </React.StrictMode>
);

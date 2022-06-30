import React from 'react';
import './exercise-index.css';
import Exercise from './components/Exercise';
import {createRoot} from 'react-dom/client';
import './libs/tools.js';

const container = document.getElementById('exercise');
const root = createRoot(container);

root.render(
    <React.StrictMode>
      <Exercise></Exercise>
    </React.StrictMode>
);

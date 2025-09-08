import React from "react";
import "./exercise-index.css";
import { createRoot } from "react-dom/client";
import "./styles/tailwindcss_base.css";
import Exercise from "Components/Exercise";

const container = document.getElementById("app");
const root = createRoot(container!);

root.render(
  <React.StrictMode>
    <Exercise />
  </React.StrictMode>
);

import React from "react";
import "./index.css";
import App from "./App";
import { createRoot } from "react-dom/client";
import { createBrowserRouter } from "react-router";
import { RouterProvider } from "react-router/dom";
import { Home, projectLoader, Studio } from "Routes";

const router = createBrowserRouter([
  {
    path: "/academy",
    Component: App,
    children: [
      { index: true, Component: Home },
      { path: "studio/:proj_id", Component: Studio, loader: projectLoader },
    ],
  },
]);

const container = document.getElementById("root");
const root = createRoot(container!);
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
  </React.StrictMode>
);

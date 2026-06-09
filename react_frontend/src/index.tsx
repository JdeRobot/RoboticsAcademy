import React from "react";
import "./index.css";
import App from "./App";
import { createRoot } from "react-dom/client";
import { createBrowserRouter } from "react-router";
import { RouterProvider } from "react-router/dom";
import { ErrorStudio, Home, projectLoader, Studio } from "Routes";

// Clear stored state on app startup to ensure fresh state on every launch
// This removes theme preferences, search filters, and other cached UI state
const clearStoredState = () => {
  // Clear localStorage items (theme preferences)
  window.localStorage.removeItem("themeType");

  // Clear sessionStorage items (search/filter state)
  window.sessionStorage.removeItem("ra_home_filters_v1");
  window.sessionStorage.removeItem("ra_home_search_v1");
};

// Execute immediately before React mounts
clearStoredState();

const router = createBrowserRouter([
  {
    path: "/academy",
    Component: App,
    children: [
      { index: true, Component: Home },
      {
        path: "studio/:proj_id",
        Component: Studio,
        loader: projectLoader,
        ErrorBoundary: ErrorStudio,
      },
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

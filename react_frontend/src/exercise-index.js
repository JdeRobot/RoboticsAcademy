import React, { lazy } from "react";
import "./exercise-index.css";
import { createRoot } from "react-dom/client";
import "./libs/tools.js";
import { flushSync } from "react-dom";
import CommsManager from "./libs/comms_manager";

import "./styles/tailwindcss_base.css";

window.RoboticsExerciseComponents = (function () {
  let components = [];

  const createElement = function (element) {
    const children = Array.from(element.childNodes).map((child) =>
      createElement(child)
    );
    return createElement(element, {}, children);
  };

  const renderComponentNew = function (
    component,
    container_id,
    properties,
    children_elements
  ) {
    const container = document.getElementById(container_id);
    properties["key"] = container_id;
    let element = React.createElement(component, properties, children_elements);
    components.push(element);
    return element;
  };

  const renderImportNew = async function (element, parent_is_root) {
    const { component, dom_id, properties, children } = element;

    const is_root = component === "root";

    const child_elements = await Promise.all(
      children.map(async (child) => {
        return await renderImportNew(child, is_root);
      })
    );

    const path = component.split("/");
    let rendered_component = {};

    if (is_root) return rendered_component;

    if (path[0] === "exercise") {
      const component_path = `${path.slice(2).join("/")}.js`;
      try {
        rendered_component = await import(`exercises/${component_path}`).then(
          (component) => {
            return renderComponentNew(
              component.default,
              dom_id,
              properties,
              child_elements
            );
          }
        );
      } catch (error) {
        console.warn(
          `⚠️ Failed to load exercise component: exercises/${component_path}`,
          error
        );
        // Optional: show a fallback message or just return an empty object
        return renderComponentNew(
          () =>
            React.createElement(
              "div",
              { className: "text-red-600 p-2" },
              `Missing component: ${component_path}`
            ),
          dom_id,
          properties,
          child_elements
        );
      }
    } else {
      try {
        rendered_component = await import(`/${component}.js`).then(
          (component) => {
            return renderComponentNew(
              component.default,
              dom_id,
              properties,
              child_elements
            );
          }
        );
      } catch (error) {
        console.warn(`⚠️ Failed to load component: /${component}.js`, error);
        return renderComponentNew(
          () =>
            React.createElement(
              "div",
              { className: "text-red-600 p-2" },
              `Missing component: ${component}`
            ),
          dom_id,
          properties,
          child_elements
        );
      }
    }

    if (parent_is_root) {
      const root = createRoot(document.getElementById(dom_id));
      flushSync(() => {
        root.render(rendered_component);
      });
    }

    return rendered_component;
  };

  const render = async function (rootRenderer) {
    await renderImportNew(rootRenderer, false);

    for (var i = 0, length = onLoadSuscribers.length; i < length; i++) {
      onLoadSuscribers[i]();
    }
  };

  const onLoadSuscribers = [];

  const ramHost = window.location.hostname;
  const ramPort = 7163;
  const ramManager = CommsManager(`ws://${ramHost}:${ramPort}`);

  return {
    components: components,
    render: render,
    commsManager: ramManager,
    suscribeOnLoad: (callback) => onLoadSuscribers.push(callback),
  };
})();

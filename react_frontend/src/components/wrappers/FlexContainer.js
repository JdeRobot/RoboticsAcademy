import classNames from "classnames";
import React, { useRef } from "react";

import "../../styles/wrappers/FlexContainer.css";

const FlexContainer = (props) => {
  const containerRef = useRef();
  const separatorRef = useRef();
  const firstChildRef = useRef();

  var containerClass = classNames({
    "flex-container": true,
    "flex-container-row": props.row,
    "flex-container-column": !props.row,
  });

  var separatorClass = classNames({
    fa: true,
    "fa-ellipsis-v": props.row,
    "fa-ellipsis-h": !props.row,
  });

  React.useEffect(() => {
    separatorRef.current.addEventListener("mousedown", onMouseDown, false);
    return () => {
      separatorRef.current.removeEventListener("mousedown", onMouseDown, false);
    };
  });

  const onMouseDown = (e) => {
    if (e.which === 1) {
      containerRef.current.addEventListener("mousemove", onMouseMove);
      window.addEventListener("mouseup", onMouseUp, true);
    }
  };

  const onMouseUp = (e) => {
    if (e.which === 1) {
      containerRef.current.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("mouseup", onMouseUp, true);
      console.log(e);
    }
  };

  const onMouseMove = (e) => {
    if (e.currentTarget !== containerRef.current) return;
    const bounds = e.currentTarget.getBoundingClientRect();
    const x = e.clientX - bounds.left;
    const y = e.clientY - bounds.top;
    console.log(
      `Bounds: (${bounds.left}, ${bounds.top}) Client: (${e.clientX}, ${e.clientY}) Coordinates: (${x}, ${y})`
    );

    if (props.row) {
      firstChildRef.current.style.width = x + "px";
    } else {
      firstChildRef.current.style.height = y + "px";
    }
  };

  return (
    <div ref={containerRef} className={containerClass}>
      <div ref={firstChildRef} className={"flex-container-first"}>
        {props.children[0]}
      </div>
      <div ref={separatorRef} className={"flex-container-divider"}>
        <i className={separatorClass}></i>
      </div>
      <div className={"flex-container-last"}>{props.children.slice(1)}</div>
    </div>
  );
};

export default FlexContainer;

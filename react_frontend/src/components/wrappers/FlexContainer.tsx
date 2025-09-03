import classNames from "classnames";
import React, { useRef, useEffect, ReactNode } from "react";

import "../../styles/wrappers/FlexContainer.css";

interface FlexContainerProps {
  row?: boolean;
  console?: boolean;
  children: ReactNode | ReactNode[];
}

const FlexContainer: React.FC<FlexContainerProps> = (props) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const separatorRef = useRef<HTMLDivElement>(null);
  const firstChildRef = useRef<HTMLDivElement>(null);
  const iframeCoverRef = useRef<HTMLDivElement>(null);

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

  var consoleClass = classNames({
    "console-on": props.console,
    "flex-container-first": !props.console,
  });

  useEffect(() => {
    const sepCurrent = separatorRef.current;
    if (!sepCurrent) return;
  
    sepCurrent.addEventListener("mousedown", onMouseDown, false);
    return () => {
      sepCurrent.removeEventListener("mousedown", onMouseDown, false);
      containerRef.current?.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("mouseup", onMouseUp, true);
    };
  }, [props.row]);
  
  const onMouseDown = (e: MouseEvent) => {
    if (e.which === 1) {
      containerRef.current!.addEventListener("mousemove", onMouseMove);
      window.addEventListener("mouseup", onMouseUp, true);
      if (iframeCoverRef.current) {
        iframeCoverRef.current.style.display = "block";
      }
    }
  };

  const onMouseUp = (e: MouseEvent) => {
    if (e.which === 1) {
      containerRef.current!.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("mouseup", onMouseUp, true);
      if (iframeCoverRef.current) {
        iframeCoverRef.current.style.display = "none";
      }
      console.log(e);
    }
  };

  const onMouseMove = (e: MouseEvent) => {
    if (e.currentTarget !== containerRef.current) return;
    const bounds = (e as any).currentTarget!.getBoundingClientRect();
    const x = e.clientX - bounds.left;
    const y = e.clientY - bounds.top;
    console.log(
      `Bounds: (${bounds.left}, ${bounds.top}) Client: (${e.clientX}, ${e.clientY}) Coordinates: (${x}, ${y})`
    );
    
    if (props.row) {
        if (firstChildRef.current) firstChildRef.current.style.width = `${x}px`;
      } else {
        if (firstChildRef.current) firstChildRef.current.style.height = `${y}px`;
      }
    };

  return (
    <div ref={containerRef} className={containerClass}>
      <div ref={firstChildRef} className={consoleClass}>
        {Array.isArray(props.children) ? props.children[0] : props.children}
      </div>
      <div
        ref={iframeCoverRef}
        style={{
          position: "absolute",
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          background: "rgba(0,0,0,0)",
          display: "none",
          zIndex: 9999,
        }}
      ></div>
      <div ref={separatorRef} className={"flex-container-divider"}>
        <i className={separatorClass}></i>
      </div>
      <div className={"flex-container-last"}>{Array.isArray(props.children) ? props.children.slice(1) : null}</div>
    </div>
  );
};

export default FlexContainer;

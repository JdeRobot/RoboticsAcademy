import * as React from "react";
import {Fragment} from "react";

import "./css/LabyrinthEscape.css";

const LabyrinthEscape = (props) => {
  return (
    <Fragment>
    {props.children}
    </Fragment>
  );
};

export default LabyrinthEscape;
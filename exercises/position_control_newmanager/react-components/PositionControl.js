import * as React from "react";
import {Fragment} from "react";

import "./css/PositionControl.css";

const PositionControl = (props) => {
  return (
    <Fragment>
    {props.children}
    </Fragment>
  );
};

export default PositionControl;
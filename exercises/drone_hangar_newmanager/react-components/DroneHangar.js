import * as React from "react";
import {Fragment} from "react";

import "./css/DroneHangar.css";

const DroneHangar = (props) => {
  return (
    <Fragment>
    {props.children}
    </Fragment>
  );
};

export default DroneHangar;
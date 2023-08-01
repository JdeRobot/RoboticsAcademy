import * as React from "react";
import {Fragment} from "react";

import "./css/VisualLander.css";

const VisualLander = (props) => {
  return (
    <Fragment>
    {props.children}
    </Fragment>
  );
};

export default VisualLander;
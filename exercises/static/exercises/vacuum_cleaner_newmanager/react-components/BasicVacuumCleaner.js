import * as React from "react";
import {Fragment} from "react";

import "./css/BasicVacuumCleaner.css";

const BasicVacuumCleaner = (props) => {
  return (
    <Fragment>
    {props.children}
    </Fragment>
  );
};

export default BasicVacuumCleaner;
import * as React from "react";
import {Fragment} from "react";

import "./css/BasicVacuumCleaner.css";

const LocVacuumCleaner = (props) => {
  return (
    <Fragment>
    {props.children}
    </Fragment>
  );
};

export default LocVacuumCleaner;
import * as React from "react";
import {Fragment} from "react";

import "./css/GlobalNavigationRR.css";

const GlobalNavigation = (props) => {
  return (
    <Fragment>
    {props.children}
    </Fragment>
  );
};

export default GlobalNavigation;
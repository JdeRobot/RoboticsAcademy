import * as React from "react";
import {Fragment} from "react";

import "./css/DigClassMain.css";

const DigClassMain = (props) => {
  return (
    <Fragment>
    {props.children}
    </Fragment>
  );
};

export default DigClassMain;
import * as React from "react";
import PropTypes from "prop-types";
export default function LocalMap(props) {
  const { localMapRef } = React.useContext(props.context);
  return <canvas id={"local-map"} ref={localMapRef} />;
}

LocalMap.propTypes = {
  context: PropTypes.any,
};

import * as React from "react";
import PropTypes from "prop-types";

function GazeboViewer() {
  return (
    <>
      <iframe
        id={"iframe"}
        style={{
          width: "100%",
          height: "400px",
        }}
        src={"http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true"}
      />
    </>
  );
}
GazeboViewer.propTypes = {
  context: PropTypes.any,
};

export default GazeboViewer;

import * as React from "react";
import PropTypes from "prop-types";

function VncConsoleViewer() {
  return (
    <>
      <iframe
        id={"console-vnc"}
        style={{
          width: "100%",
          height: "250px",
        }}
        src={"http://127.0.0.1:1108/vnc.html?resize=remote&autoconnect=true"}
      />
    </>
  );
}
VncConsoleViewer.propTypes = {
  context: PropTypes.any,
};

export default VncConsoleViewer;

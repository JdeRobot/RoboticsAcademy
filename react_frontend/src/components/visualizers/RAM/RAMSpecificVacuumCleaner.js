import * as React from "react";
import "../../../styles/birdsEye.css";
import PropTypes from "prop-types";

export default function SpecificVacuumCleaner() {
  return (
    <canvas
      style={{
        backgroundImage:
          "url('/static/exercises/vacuum_cleaner_newmanager/resources/images/mapgrannyannie.png')",
        margin: "20px",
        border: "2px solid #d3d3d3",
        backgroundRepeat: "no-repeat",
        backgroundSize: "100% 100%",
        width: "80%",
        height: "80%",
      }}
    />
  );
}

SpecificVacuumCleaner.propTypes = {
  circuit: PropTypes.string,
};

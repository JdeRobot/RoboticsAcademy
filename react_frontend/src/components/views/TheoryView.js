import * as React from "react";
import PropTypes from "prop-types";

const TheoryView = (props) => {
  const theoryRef = React.useRef(null);
  const [iFrameHeight, setIframeHeight] = React.useState("1000px");
  const loadIframeHeight = () => {
    setIframeHeight(
      theoryRef.current.contentWindow.document.body.scrollHeight + "px"
    );
  };
  return (
    <iframe
      src={props.url}
      id="theory-view"
      width={"100%"}
      ref={theoryRef}
      height={iFrameHeight}
      onLoad={loadIframeHeight}
      style={{
        width: "100%",
        overflow: "auto",
      }}
    />
  );
};
TheoryView.propTypes = {
  url: PropTypes.string,
};

export default TheoryView;

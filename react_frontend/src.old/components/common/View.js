import * as React from "react";
import TheoryView from "../views/TheoryView";
import ForumView from "../views/ForumView";
import ViewContext from "../../contexts/ViewContext";
import PropTypes from "prop-types";

function View(props) {
  const { theoryMode, codeMode, forumMode } = React.useContext(ViewContext);

  return (
    <>
      {theoryMode && <TheoryView url={props.url} />}
      {codeMode && props.exerciseId}
      {forumMode && <ForumView />}
    </>
  );
}

View.propTypes = {
  url: PropTypes.string,
  exerciseId: PropTypes.node,
};

export default View;

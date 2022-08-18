import * as React from "react";
import TheoryView from "./TheoryView";
import ExerciseView from "./ExerciseView";
import ForumView from "./ForumView";
import ViewContext from "../../contexts/ViewContext";
import InfoModalView from "./InfoModalView";
import PropTypes from "prop-types";

function View(props) {
  const { theoryMode, codeMode, forumMode } = React.useContext(ViewContext);
  return (
    <>
      {theoryMode && <TheoryView url={props.url} />}
      {codeMode && <ExerciseView />}
      {forumMode && <ForumView />}
      <InfoModalView />
    </>
  );
}

View.propTypes = {
  url: PropTypes.string,
};

export default View;

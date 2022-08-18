import * as React from "react";
import TheoryView from "./TheoryView";
import ExerciseView from "./ExerciseView";
import ForumView from "./ForumView";
import ViewContext from "../../contexts/ViewContext";
import InfoModalView from "./InfoModalView";
function View(url) {
  const { theoryMode, codeMode, forumMode } = React.useContext(ViewContext);
  return (
    <>
      {theoryMode && <TheoryView url={url} />}
      {codeMode && <ExerciseView />}
      {forumMode && <ForumView />}
      <InfoModalView />
    </>
  );
}

export default View;

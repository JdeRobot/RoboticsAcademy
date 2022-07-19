import * as React from "react";
import TheoryView from "./TheoryView";
import ExerciseView from "./ExerciseView";
import ForumView from "./ForumView";
import ViewContext from "../../contexts/ViewContext";
function View() {
  const { theoryMode, codeMode, forumMode } = React.useContext(ViewContext);
  return (
    <>
      {theoryMode && (
        <TheoryView
          url={
            "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/"
          }
        />
      )}
      {codeMode && <ExerciseView />}
      {forumMode && <ForumView />}
    </>
  );
}

export default View;

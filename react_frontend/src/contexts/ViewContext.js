import { createContext, useState } from "react";
import PropTypes from "prop-types";
import React from "react";

const ViewContext = createContext();

export function ViewProvider({ children }) {
  const [theoryMode, setTheoryMode] = useState(false);
  const [codeMode, setCodeMode] = useState(true);
  const [forumMode, setForumMode] = useState(false);
  const openTheory = () => {
    if (!theoryMode) {
      setTheoryMode(true);
      setCodeMode(false);
      setForumMode(false);
    }
  };
  const openExercise = () => {
    if (!codeMode) {
      setTheoryMode(false);
      setCodeMode(true);
      setForumMode(false);
    }
  };
  const openForum = () => {
    if (!forumMode) {
      setTheoryMode(false);
      setCodeMode(false);
      setForumMode(true);
    }
  };

  return (
    <ViewContext.Provider
      value={{
        theoryMode,
        codeMode,
        forumMode,
        openTheory,
        openForum,
        openExercise,
      }}
    >
      {children}
    </ViewContext.Provider>
  );
}
ViewProvider.propTypes = {
  children: PropTypes.element,
};

export default ViewContext;

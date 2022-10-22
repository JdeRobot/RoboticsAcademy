import { createContext, useState } from "react";
import PropTypes from "prop-types";
import React from "react";
const HomepageContext = createContext();

export function HomepageProvider({ children }) {
  const [inputText, setInputText] = useState("");

  const getSearchBarText = () => inputText;

  const setSearchBarText = (text) => {
    setInputText(text);
  };

  return (
    <HomepageContext.Provider
      value={{
        getSearchBarText,
        setSearchBarText,
      }}
    >
      {children}
    </HomepageContext.Provider>
  );
}

HomepageProvider.propTypes = {
  children: PropTypes.element,
};

export default HomepageContext;

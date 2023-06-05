import { createContext, useState } from "react";
import PropTypes from "prop-types";
import React from "react";
const HomepageContext = createContext();

export function HomepageProvider({ children }) {
  const [inputText, setInputText] = useState("");
  const [filterItemsList, setFilterItemsList] = useState(["name", "tags"]);

  const getSearchBarText = () => inputText;

  const setSearchBarText = (text) => {
    setInputText(text);
  };
  const getFilterItemsList = () => filterItemsList;
  const appendFilterItem = (item) => {
    setFilterItemsList(
      filterItemsList.includes(item)
        ? filterItemsList.filter((i) => i !== item)
        : [...filterItemsList, item]
    );
  };

  return (
    <HomepageContext.Provider
      value={{
        getSearchBarText,
        setSearchBarText,
        appendFilterItem,
        getFilterItemsList,
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

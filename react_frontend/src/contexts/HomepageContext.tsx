import { createContext, ReactNode, useContext, useState } from "react";
import { Filters } from "Types/exercises";
import React from "react";

export interface HomepageContextType {
  getSearchBarText: () => string;
  setSearchBarText: (text: string) => void;
  appendFilterItem: (item: Filters) => void;
  getFilterItemsList: () => Filters[];
  setFilterItemsList: (items: Filters[]) => void;
}

const HomepageContext = createContext<HomepageContextType>({
  getSearchBarText: () => "",
  setSearchBarText: () => {},
  appendFilterItem: () => {},
  getFilterItemsList: () => [],
  setFilterItemsList: () => {},
});

export const useHomepage = () => useContext(HomepageContext);

export function HomepageProvider({ children }: { children?: ReactNode }) {
  const [inputText, setInputText] = useState("");
  const [filterItemsList, setFilterItemsList] = useState<Filters[]>([
    "name",
    "tags",
  ]);

  const getSearchBarText = () => inputText;

  const setSearchBarText = (text: string) => {
    setInputText(text);
  };

  const getFilterItemsList = () => filterItemsList;

  const appendFilterItem = (item: Filters) => {
    setFilterItemsList((prev) =>
      prev.includes(item) ? prev.filter((i) => i !== item) : [...prev, item],
    );
  };

  return (
    <HomepageContext.Provider
      value={{
        getSearchBarText,
        setSearchBarText,
        appendFilterItem,
        getFilterItemsList,
        setFilterItemsList,
      }}
    >
      {children}
    </HomepageContext.Provider>
  );
}

export default HomepageContext;

import { createContext, ReactNode, useContext, useState } from "react";
import { Filters } from "src/types/exercises";

export interface HomepageContextType {
  getSearchBarText: () => string;
  setSearchBarText: (text: string) => void;
  appendFilterItem: (item: Filters) => void;
  getFilterItemsList: () => Filters[];
}

const HomepageContext = createContext<HomepageContextType>({
  getSearchBarText: () => {return ""},
  setSearchBarText: (text: string) => {},
  appendFilterItem: (item: Filters) => {},
  getFilterItemsList: () => {return []},
});

export const useHomepage = () => useContext(HomepageContext);

export function HomepageProvider({ children }: {children?: ReactNode;}) {
  const [inputText, setInputText] = useState("");
  const [filterItemsList, setFilterItemsList] = useState<Filters[]>(["name", "tags"]);

  const getSearchBarText = () => inputText;

  const setSearchBarText = (text: string) => {
    setInputText(text);
  };
  const getFilterItemsList = () => filterItemsList;
  const appendFilterItem = (item: Filters) => {
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

export default HomepageContext;

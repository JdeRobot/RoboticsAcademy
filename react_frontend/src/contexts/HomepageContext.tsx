import { createContext, ReactNode, useState } from "react";


const HomepageContext = createContext({
  getSearchBarText: () => {},
  setSearchBarText: (text: string) => {},
  getFilterItemsList: () => {},
  appendFilterItem: (item: string) => {},
});

export function HomepageProvider({ children }: {children?: ReactNode;}) {
  const [inputText, setInputText] = useState("");
  const [filterItemsList, setFilterItemsList] = useState(["name", "tags"]);

  const getSearchBarText = () => inputText;

  const setSearchBarText = (text: string) => {
    setInputText(text);
  };
  const getFilterItemsList = () => filterItemsList;
  const appendFilterItem = (item: string) => {
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

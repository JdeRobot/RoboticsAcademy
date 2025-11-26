import * as React from "react";
import { styled, alpha } from "@mui/material/styles";
import SearchIcon from "@mui/icons-material/Search";
import FilterListIcon from "@mui/icons-material/FilterList";
import {
  Box,
  Checkbox,
  IconButton,
  InputBase,
  Menu,
  MenuItem,
} from "@mui/material";
import { useHomepage } from "Contexts/HomepageContext";
import { useState, useEffect } from "react";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { Filters } from "src/types/exercises";

// SessionStorage keys
const FILTER_STORAGE_KEY = "ra_home_filters_v1";
const SEARCH_STORAGE_KEY = "ra_home_search_v1";

// Estilos
const Search = styled("div")(({ roundness }: { roundness: number }) => ({
  position: "relative",
  borderRadius: roundness + "px",
  backgroundColor: alpha("#fff", 0.15),
  "&:hover": {
    backgroundColor: alpha("#fff", 0.25),
  },
  marginRight: 16 + "px",
  marginLeft: "24px",
  width: "auto",
}));

const SearchIconWrapper = styled("div")(({ theme }) => ({
  padding: theme.spacing(0, 2),
  height: "100%",
  position: "absolute",
  pointerEvents: "none",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
}));

const StyledInputBase = styled(InputBase)(
  ({ textColor }: { textColor: string }) => ({
    "& .MuiInputBase-input": {
      padding: "8px 8px 8px 0",
      paddingLeft: `calc(1em + 32px)`,
      transition: "width 300ms cubic-bezier(0.4, 0, 0.2, 1) 0ms",
      width: "20ch",
      color: textColor,
    },
  })
);

const StyledMenu = styled(Menu)(
  ({
    bgColor,
    textColor,
    checkboxColor,
    hoverColor,
    roundness,
  }: {
    bgColor: string;
    textColor: string;
    checkboxColor: string;
    hoverColor: string;
    roundness: number;
  }) => ({
    "& .MuiPaper-root": {
      border: "1px solid black",
      borderRadius: roundness + "px",
      backgroundColor: bgColor,
      "& .MuiMenuItem-root": {
        color: textColor,
        "&:hover": {
          backgroundColor: hoverColor,
        },
        "& .MuiSvgIcon-root": {
          color: checkboxColor,
        },
        "& .Mui-disabled": {
          "& .MuiSvgIcon-root": {
            opacity: "30%",
          },
        },
      },
    },
  })
);

// Filter persistence structure
type FilterState = {
  tags: boolean;
  description: boolean;
  status: boolean;
};

const DEFAULT_FILTER_STATE: FilterState = {
  tags: true,
  description: false,
  status: false,
};

const filtersFromState = (state: FilterState): Filters[] => {
  const list: Filters[] = ["name"]; // name is always active
  if (state.tags) list.push("tags");
  if (state.description) list.push("description");
  if (state.status) list.push("status");
  return list;
};

const FilterMenu = () => {
  const { appendFilterItem, setFilterItemsList } = useHomepage();
  const theme = useAcademyTheme();
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const open: boolean = Boolean(anchorEl);

  const [filterState, setFilterState] =
    useState<FilterState>(DEFAULT_FILTER_STATE);

  // Restore session filter state
  useEffect(() => {
    if (typeof window === "undefined") return;

    try {
      const raw = window.sessionStorage.getItem(FILTER_STORAGE_KEY);
      if (!raw) return;

      const parsed = JSON.parse(raw) as Partial<FilterState>;
      const persisted: FilterState = {
        tags:
          typeof parsed.tags === "boolean"
            ? parsed.tags
            : DEFAULT_FILTER_STATE.tags,
        description:
          typeof parsed.description === "boolean"
            ? parsed.description
            : DEFAULT_FILTER_STATE.description,
        status:
          typeof parsed.status === "boolean"
            ? parsed.status
            : DEFAULT_FILTER_STATE.status,
      };

      setFilterState(persisted);
      setFilterItemsList(filtersFromState(persisted));
    } catch (err) {
      console.error("Failed to restore filter state", err);
    }
  }, [setFilterItemsList]);

  const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
    setAnchorEl(event.currentTarget);
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  // Toggle and persist filter values
  const handleFilterCheckboxChange = (
    e: React.ChangeEvent<HTMLInputElement>
  ) => {
    const name = e.target.name as keyof FilterState;

    setFilterState((prev) => {
      const updated: FilterState = {
        ...prev,
        [name]: !prev[name],
      };

      try {
        window.sessionStorage.setItem(
          FILTER_STORAGE_KEY,
          JSON.stringify(updated)
        );
      } catch (err) {
        console.error("Failed to save filter state", err);
      }

      return updated;
    });

    appendFilterItem(name as Filters);
  };

  return (
    <>
      <IconButton
        id="basic-button"
        aria-controls={open ? "basic-menu" : undefined}
        aria-haspopup="true"
        aria-expanded={open ? "true" : undefined}
        onClick={handleClick}
      >
        <FilterListIcon htmlColor={theme.palette.text} />
      </IconButton>
      <StyledMenu
        id="basic-menu"
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
        bgColor={theme.palette.primary!}
        hoverColor={theme.palette.secondary!}
        textColor={theme.palette.text!}
        checkboxColor={theme.palette.text!}
        roundness={theme.roundness!}
      >
        {/* Name always active and not persisted */}
        <MenuItem>
          <Checkbox defaultChecked disabled size="small" name="name" />
          Name
        </MenuItem>
        <MenuItem>
          <Checkbox
            size="small"
            name="tags"
            checked={filterState.tags}
            onChange={handleFilterCheckboxChange}
          />
          Tags
        </MenuItem>
        <MenuItem>
          <Checkbox
            size="small"
            name="description"
            checked={filterState.description}
            onChange={handleFilterCheckboxChange}
          />
          Description
        </MenuItem>
        <MenuItem>
          <Checkbox
            size="small"
            name="status"
            checked={filterState.status}
            onChange={handleFilterCheckboxChange}
          />
          Status
        </MenuItem>
      </StyledMenu>
    </>
  );
};

const SearchBar = () => {
  const { setSearchBarText } = useHomepage();
  const theme = useAcademyTheme();

  const [searchValue, setSearchValue] = useState<string>("");

  // Restore session search text
  useEffect(() => {
    if (typeof window === "undefined") return;

    try {
      const saved = window.sessionStorage.getItem(SEARCH_STORAGE_KEY);
      if (saved) {
        setSearchValue(saved);
        setSearchBarText(saved.toLowerCase());
      }
    } catch (err) {
      console.error("Failed to restore search text", err);
    }
  }, [setSearchBarText]);

  // Update sessionStorage + context
  const inputHandler = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = e.target.value;
    setSearchValue(value);

    const lowerCase = value.toLowerCase();
    setSearchBarText(lowerCase);

    try {
      window.sessionStorage.setItem(SEARCH_STORAGE_KEY, value);
    } catch (err) {
      console.error("Failed to save search text", err);
    }
  };

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "row",
        maxHeight: 40,
      }}
    >
      <Search roundness={theme.roundness!}>
        <SearchIconWrapper>
          <SearchIcon htmlColor={theme.palette.text} />
        </SearchIconWrapper>
        <StyledInputBase
          placeholder="Search…"
          value={searchValue}
          onChange={inputHandler}
          textColor={theme.palette.text!}
          inputProps={{ "aria-label": "search" }}
        />
        <FilterMenu />
      </Search>
    </Box>
  );
};

export default SearchBar;


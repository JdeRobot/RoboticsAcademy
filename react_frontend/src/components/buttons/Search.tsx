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
import { useState } from "react";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";

// Estilos
const Search = styled("div")(({ theme }) => ({
  position: "relative",
  borderRadius: theme.shape.borderRadius,
  backgroundColor: alpha(theme.palette.common.white, 0.15),
  "&:hover": {
    backgroundColor: alpha(theme.palette.common.white, 0.25),
  },
  marginRight: theme.spacing(2),
  marginLeft: 0,
  width: "100%",
  [theme.breakpoints.up("sm")]: {
    marginLeft: theme.spacing(3),
    width: "auto",
  },
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

const StyledInputBase = styled(InputBase)(({ theme }) => ({
  "& .MuiInputBase-input": {
    padding: theme.spacing(1, 1, 1, 0),
    paddingLeft: `calc(1em + ${theme.spacing(4)})`,
    transition: theme.transitions.create("width"),
    width: "100%",
    [theme.breakpoints.up("md")]: {
      width: "20ch",
    },
  },
}));

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

// Componente FilterMenu
const FilterMenu = () => {
  const { appendFilterItem } = useHomepage();
  const theme = useAcademyTheme();
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const open: boolean = Boolean(anchorEl);

  const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
    setAnchorEl(event.currentTarget);
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  const handleFilterList = (e: any) => {
    const item = e.target.name;
    appendFilterItem(item);
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
        bgColor={theme.palette.primary}
        hoverColor={theme.palette.secondary}
        textColor={theme.palette.text}
        checkboxColor={theme.palette.text}
        roundness={theme.roundness}
      >
        <MenuItem onClick={handleFilterList}>
          <Checkbox defaultChecked disabled size="small" name="name" />
          Name
        </MenuItem>
        <MenuItem>
          <Checkbox
            defaultChecked
            size="small"
            onClick={handleFilterList}
            name="tags"
          />
          Tags
        </MenuItem>
        <MenuItem>
          <Checkbox
            size="small"
            onClick={handleFilterList}
            name="description"
          />
          Description
        </MenuItem>
        <MenuItem>
          <Checkbox size="small" onClick={handleFilterList} name="status" />
          Status
        </MenuItem>
      </StyledMenu>
    </>
  );
};

// Componente principal SearchBar
const SearchBar = () => {
  const { setSearchBarText } = useHomepage();
  const theme = useAcademyTheme();

  const inputHandler = (e: React.ChangeEvent<HTMLInputElement>) => {
    const lowerCase = e.target.value.toLowerCase();
    setSearchBarText(lowerCase);
  };

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "row",
        maxHeight: 40,
      }}
    >
      <Search>
        <SearchIconWrapper>
          <SearchIcon htmlColor={theme.palette.text} />
        </SearchIconWrapper>
        <StyledInputBase
          placeholder="Search…"
          onChange={inputHandler}
          color={theme.palette.text}
          inputProps={{ "aria-label": "search" }}
        />
        <FilterMenu />
      </Search>
    </Box>
  );
};

export default SearchBar;

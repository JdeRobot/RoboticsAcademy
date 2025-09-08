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
        bgColor={theme.palette.primary!}
        hoverColor={theme.palette.secondary!}
        textColor={theme.palette.text!}
        checkboxColor={theme.palette.text!}
        roundness={theme.roundness!}
      >
        <MenuItem>
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
      <Search roundness={theme.roundness!}>
        <SearchIconWrapper>
          <SearchIcon htmlColor={theme.palette.text} />
        </SearchIconWrapper>
        <StyledInputBase
          placeholder="Search…"
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

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
import HomepageContext from "../contexts/HomepageContext";

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
  color: "inherit",
  "& .MuiInputBase-input": {
    padding: theme.spacing(1, 1, 1, 0),
    // vertical padding + font size from searchIcon
    paddingLeft: `calc(1em + ${theme.spacing(4)})`,
    transition: theme.transitions.create("width"),
    width: "100%",
    [theme.breakpoints.up("md")]: {
      width: "20ch",
    },
  },
}));

const FilterMenu = () => {
  const { appendFilterItem } = React.useContext(HomepageContext);
  const [anchorEl, setAnchorEl] = React.useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };
  const handleFilterList = (e) => {
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
        <FilterListIcon />
      </IconButton>
      <Menu
        id="basic-menu"
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
      >
        <MenuItem onClick={handleFilterList}>
          <Checkbox defaultChecked disabled size="small" name={"name"} /> Name
        </MenuItem>
        <MenuItem>
          <Checkbox
            defaultChecked
            size="small"
            onClick={handleFilterList}
            name={"tags"}
          />
          tags
        </MenuItem>
        <MenuItem>
          <Checkbox
            size="small"
            onClick={handleFilterList}
            name={"description"}
          />{" "}
          description
        </MenuItem>
        <MenuItem>
          <Checkbox size="small" onClick={handleFilterList} name={"status"} />{" "}
          Status
        </MenuItem>
      </Menu>
    </>
  );
};
const SearchBar = () => {
  const { setSearchBarText } = React.useContext(HomepageContext);
  let inputHandler = (e) => {
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
          <SearchIcon />
        </SearchIconWrapper>
        <StyledInputBase
          placeholder="Search…"
          onChange={inputHandler}
          inputProps={{ "aria-label": "search" }}
        />
        <FilterMenu />
      </Search>
    </Box>
  );
};

export default SearchBar;

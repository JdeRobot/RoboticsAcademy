import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Box from "@mui/material/Box";
import Divider from "@mui/material/Divider";
import Drawer from "@mui/material/Drawer";
import IconButton from "@mui/material/IconButton";
import List from "@mui/material/List";
import ListItemButton from "@mui/material/ListItemButton";
import MenuIcon from "@mui/icons-material/Menu";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import Button from "@mui/material/Button";
import ForumIcon from "@mui/icons-material/Forum";
import { useState } from "react";
import SearchBar from "./SearchBar";

const drawerWidth = 240;

export default function DrawerAppBar(props) {
  const { window_ } = props;

  const [mobileOpen, setMobileOpen] = useState(false);

  const handleDrawerToggle = () => {
    setMobileOpen(!mobileOpen);
  };
  const goToForum = () => {
    const url = "https://forum.jderobot.org/c/english/roboticsacademy/12";
    window.location.href = url;
  };

  const drawer = (
    <Box sx={{ textAlign: "center" }}>
      <Typography variant="h6" sx={{ my: 2 }}>
        Robotics Academy by JdeRobot
      </Typography>
      <Divider />
      <SearchBar />
      <List>
        <ListItemButton sx={{ textAlign: "center" }} onClick={goToForum}>
          Forum
        </ListItemButton>
      </List>
    </Box>
  );

  const container =
    window_ !== undefined ? () => window().document.body : undefined;

  return (
    <Box sx={{ display: "flex" }} mb={10}>
      <AppBar component="nav">
        <Toolbar>
          <IconButton
            color="inherit"
            aria-label="open drawer"
            edge="start"
            onClick={handleDrawerToggle}
            sx={{ mr: 2, display: { sm: "none" } }}
          >
            <MenuIcon />
          </IconButton>
          <Typography
            variant="h6"
            component="div"
            sx={{ flexGrow: 1, display: { xs: "none", sm: "block" } }}
          >
            Robotics Academy by JdeRobot
          </Typography>
          <SearchBar />
          <Button
            variant="outlined"
            sx={{ display: { xs: "none", sm: "block" }, color: "#fff" }}
            startIcon={<ForumIcon />}
            onClick={goToForum}
          >
            Forum
          </Button>
        </Toolbar>
      </AppBar>
      <Box component="nav">
        <Drawer
          container={container}
          variant="temporary"
          open={mobileOpen}
          onClose={handleDrawerToggle}
          ModalProps={{
            keepMounted: true, // Better open performance on mobile.
          }}
          sx={{
            display: { xs: "block", sm: "none" },
            "& .MuiDrawer-paper": {
              boxSizing: "border-box",
              width: drawerWidth,
            },
          }}
        >
          {drawer}
        </Drawer>
      </Box>
    </Box>
  );
}

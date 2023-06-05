import * as React from "react";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableHead from "@mui/material/TableHead";
import TableRow from "@mui/material/TableRow";
import Paper from "@mui/material/Paper";
import { Box, Typography } from "@mui/material";
import PropTypes from "prop-types";

function FrequencyMenuHelper(props) {
  const { frequencyRows } = React.useContext(props.context);

  return (
    <TableContainer component={Paper}>
      <Table>
        <TableHead>
          <TableRow>
            <TableCell> Key </TableCell>
            <TableCell align="right"> Value </TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {frequencyRows.map((row) => (
            <TableRow key={row.key}>
              <TableCell>{row.key}</TableCell>
              <TableCell align="right">{row.value}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );
}

FrequencyMenuHelper.propTypes = {
  context: PropTypes.any,
};

FrequencyMenu.propTypes = {
  context: PropTypes.any,
};

export default function FrequencyMenu(props) {
  return (
    <Box
      sx={{
        m: 3,
        p: 2,
        display: "flex",
        flexDirection: "column",
        border: "2px solid",
        justifyContent: "space-around",
        alignItems: "center",
      }}
    >
      <Typography>Frequency Menu</Typography>
      <FrequencyMenuHelper context={props.context} />
    </Box>
  );
}

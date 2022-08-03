import * as React from "react";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableHead from "@mui/material/TableHead";
import TableRow from "@mui/material/TableRow";
import Paper from "@mui/material/Paper";

function createData(key, value) {
  return { key, value };
}

const rows = [
  createData("Brain Frequency (Hz)", 159),
  createData("GUI Frequency (Hz)", 237),
  createData("Simulation Real time factor", 262),
];

export default function FrequencyMenu() {
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
          {rows.map((row) => (
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

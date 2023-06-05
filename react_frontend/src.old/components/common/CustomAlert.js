import * as React from "react";
import { Snackbar } from "@mui/material";
import MuiAlert from "@mui/material/Alert";
import PropTypes from "prop-types";

const Alert = React.forwardRef(function Alert(props, ref) {
  return <MuiAlert elevation={6} ref={ref} variant="filled" {...props} />;
});

function CustomAlert(props) {
  const { alertState, alertContent } = React.useContext(props.context);
  const [snackbar, setSnackbar] = React.useState(false);
  React.useEffect(() => {
    setSnackbar(true);
  }, [alertState]);
  const handleSnackBarClose = (event, reason) => {
    if (reason === "clickaway") {
      return;
    }

    setSnackbar(false);
  };

  return (
    <div>
      {alertState.errorAlert ? (
        <Snackbar
          open={snackbar}
          autoHideDuration={6000}
          onClose={handleSnackBarClose}
        >
          <Alert severity="error">{alertContent}</Alert>
        </Snackbar>
      ) : (
        <></>
      )}
      {alertState.successAlert ? (
        <Snackbar
          open={snackbar}
          autoHideDuration={6000}
          onClose={handleSnackBarClose}
        >
          <Alert
            onClose={handleSnackBarClose}
            severity="success"
            sx={{ width: "100%" }}
          >
            {alertContent}
          </Alert>
        </Snackbar>
      ) : (
        <></>
      )}
      {alertState.warningAlert ? (
        <Snackbar
          open={snackbar}
          autoHideDuration={6000}
          onClose={handleSnackBarClose}
        >
          <Alert severity="warning">{alertContent}</Alert>
        </Snackbar>
      ) : (
        <></>
      )}
      {alertState.infoAlert ? (
        <Snackbar
          open={snackbar}
          autoHideDuration={6000}
          onClose={handleSnackBarClose}
        >
          <Alert severity="info">{alertContent}</Alert>
        </Snackbar>
      ) : (
        <></>
      )}
    </div>
  );
}

CustomAlert.propTypes = {
  context: PropTypes.any,
};

export default CustomAlert;

import * as React from "react";
import PropTypes from "prop-types";

const WebSocketContext = React.createContext();

export function WebSocketProvider({ children }) {
  const websocket_address = "127.0.0.1";
  const address_code = "ws://" + websocket_address + ":1905";
  const address_gui = "ws://" + websocket_address + ":2303";
  let ws_manager;
  // React.useEffect(() => {
  //   ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
  //   console.log(ws_manager);
  // }, []);

  return (
    <WebSocketContext.Provider
      value={{
        ws_manager,
        address_code,
        address_gui,
      }}
    >
      {children}
    </WebSocketContext.Provider>
  );
}

WebSocketProvider.propTypes = {
  children: PropTypes.node,
};

export default WebSocketContext;

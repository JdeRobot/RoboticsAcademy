const websocket_address = "127.0.0.1";
const address_code = "ws://" + websocket_address + ":1905";
const address_gui = "ws://" + websocket_address + ":2303";

const GlobalVariable = {
  simStop: "simStop",
  sendCode: "sendCode",
  running: "running",
  firstAttempt: "firstAttempt",
  simReset: "simReset",
  simResume: "simResume",
  resetRequested: "resetRequested",
  firstCodeSent: "firstCodeSent",
  swapping: "swapping",
  gazeboOn: "gazeboOn",
  gazeboToggle: "gazeboToggle",
  teleOpMode: "teleOpMode",
};
export { websocket_address, address_code, address_gui, GlobalVariable };

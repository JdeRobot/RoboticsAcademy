import { GlobalVariable } from "../constants";
let brainFreqAck = 12,
  guiFreqAck = 12;
let simStop = false,
  sendCode = false,
  running = true,
  firstAttempt = true,
  simReset = false,
  simResume = false,
  resetRequested = false,
  firstCodeSent = false,
  swapping = false,
  gazeboOn = false,
  gazeboToggle = false,
  teleOpMode = false;
let animation_id,
  image_data,
  source,
  shape,
  lap_time,
  pose,
  content,
  command_input;

function getValue(x) {
  switch (x) {
    case GlobalVariable.simStop:
      return simStop;
    case GlobalVariable.running:
      return running;
    case GlobalVariable.sendCode:
      return sendCode;
    case GlobalVariable.firstAttempt:
      return firstAttempt;
    case GlobalVariable.simReset:
      return simReset;
    case GlobalVariable.simResume:
      return simResume;
    case GlobalVariable.resetRequested:
      return resetRequested;
    case GlobalVariable.firstCodeSent:
      return firstCodeSent;
    case GlobalVariable.swapping:
      return swapping;
    case GlobalVariable.gazeboOn:
      return gazeboOn;
    case GlobalVariable.gazeboToggle:
      return gazeboToggle;
    case GlobalVariable.teleOpMode:
      return teleOpMode;
    default:
      console.log(x);
      console.error("Unassigned Value");
  }
}

function setValue(x, val) {
  switch (x) {
    case GlobalVariable.simStop:
      simStop = val;
      break;
    case GlobalVariable.running:
      running = val;
      break;
    case GlobalVariable.sendCode:
      sendCode = val;
      break;
    case GlobalVariable.firstAttempt:
      firstAttempt = val;
      break;
    case GlobalVariable.simReset:
      simReset = val;
      break;
    case GlobalVariable.simResume:
      simResume = val;
      break;
    case GlobalVariable.resetRequested:
      resetRequested = val;
      break;
    case GlobalVariable.firstCodeSent:
      firstCodeSent = val;
      break;
    case GlobalVariable.swapping:
      swapping = val;
      break;
    case GlobalVariable.gazeboOn:
      gazeboOn = val;
      break;
    case GlobalVariable.gazeboToggle:
      gazeboToggle = val;
      break;
    case GlobalVariable.teleOpMode:
      teleOpMode = val;
      break;
    default:
      console.log(x);
  }
}
// Car variables
let v = 0;
let w = 0;

export {
  brainFreqAck,
  guiFreqAck,
  simStop,
  sendCode,
  running,
  firstAttempt,
  simReset,
  simResume,
  resetRequested,
  firstCodeSent,
  swapping,
  gazeboOn,
  gazeboToggle,
  teleOpMode,
  animation_id,
  image_data,
  source,
  shape,
  lap_time,
  pose,
  content,
  command_input,
  v,
  w,
  setValue,
  getValue,
};

/* eslint-disable no-unused-vars */
import * as React from "react";
import { createContext, useState } from "react";
import { saveCode, get_novnc_size_react } from "../helpers/utils";
import { setIframe, setIframeConsole } from "../helpers/SetIframe.js";
import { drawCircle } from "../helpers/birdEye.js";
import PropTypes from "prop-types";
const ExerciseContext = createContext();
const websocket_address = "127.0.0.1";
const address_code = "ws://" + websocket_address + ":1905";
const address_gui = "ws://" + websocket_address + ":2303";
let ws_manager, websocket_code, websocket_gui;
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
  gazeboToggle = false;
let animation_id,
  image_data,
  source,
  shape,
  lap_time,
  pose,
  content,
  command_input;
// Car variables
let v = 0;
let w = 0;
export function ExerciseProvider({ children }) {
  const exercise = "follow_line";
  // connectionState - Connect, Connecting, Connected
  const [connectionState, setConnectionState] = useState("Connect");
  // launchState - Launch, Launching, Ready
  const [launchState, setLaunchState] = useState("Launch");
  const [frequency, setFrequency] = useState("0");
  const [launchLevel, setLaunchLevel] = useState(0);
  // const [gazeboToggle, setGazeboToggle] = useState(false);
  // const [gazeboOn, setGazeboOn] = useState(false);
  // const [simReset, setSimReset] = useState(false);
  // const [simStop, setSimStop] = useState(false);
  // const [simResume, setSimResume] = useState(false);
  // const [sendCode, setSendCode] = useState(false);
  // const [firstAttempt, setFirstAttempt] = useState(true);
  // const [swapping, setSwapping] = useState(false);
  // const [running, setRunning] = useState(true);
  // const [resetRequested, setResetRequested] = useState(false);
  // const [firstCodeSent, setFirstCodeSent] = useState(false);
  const [alertState, setAlertState] = useState({
    errorAlert: false,
    successAlert: false,
    infoAlert: false,
    warningAlert: false,
  });
  const [alertContent, setAlertContent] = useState("");
  const [openGazebo, setOpenGazebo] = useState(false);
  const [openConsole, setOpenConsole] = useState(false);
  const [initialPosition, setInitialPosition] = useState();
  const [playState, setPlayState] = useState(true);
  const [circuit, setCircuit] = useState("default");
  const [guiFreqValue, setGuiFreqValue] = useState(10);
  const [codeFreqValue, setCodeFreqValue] = useState(10);
  const [rtfValue, setRtfValue] = useState(0);
  const [teleopMode, setTeleopMode] = useState(false);
  const [birdEyeClass, setBirdEyeClass] = React.useState("default");
  const getCircuitValue = () => {
    return circuit;
  };

  const [editorCode, setEditorCode] = useState(`from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);

  const startSim = (step) => {
    var level = 0;
    let websockets_connected = false;

    if (step === 0) {
      setConnectionState("Connecting");
      ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    } else if (step === 1) {
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: `${level}` },
        "*"
      );
      var size = get_novnc_size_react();
      ws_manager.send(
        JSON.stringify({
          command: "open",
          exercise: exercise,
          width: size.width.toString(),
          height: size.height.toString(),
          circuit: circuit,
        })
      );
      level++;
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: `${level}` },
        "*"
      );
      ws_manager.send(JSON.stringify({ command: "Pong" }));
      console.log("start exercise");
    } else if (step === 2) {
      console.log(ws_manager);
      ws_manager.send(JSON.stringify({ command: "exit", exercise: "" }));
      stopSimulation();
    }
    console.log("RUNS this appy 1");
    ws_manager.onopen = function () {
      level++;
      console.log("Open Event");
      connectionUpdate({ connection: "manager", command: "up" }, "*");
      connectionUpdate({ connection: "exercise", command: "available" }, "*");
    };

    ws_manager.onclose = function () {
      console.log("RUNS this appy 2");
      connectionUpdate({ connection: "manager", command: "down" }, "*");
      if (!firstAttempt) {
        setAlertState({
          ...alertState,
          errorAlert: true,
          successAlert: false,
          warningAlert: false,
          infoAlert: false,
        });
        setAlertContent("Connection lost, retrying connection...");
        // alert("Connection lost, retrying connection...");
        startSim(step, circuit, websocket_address);
      } else {
        firstAttempt = false;
        // setFirstAttempt(false);
      }
    };

    ws_manager.onerror = function () {
      console.log("RUNS this appy error");

      connectionUpdate({ connection: "manager", command: "down" }, "*");
    };

    ws_manager.onmessage = function (event) {
      console.log("RUNS this appy 3");
      console.log(event.data);
      console.log(`send code --> `, sendCode);
      if (event.data.level > level) {
        level = event.data.level;
        connectionUpdate(
          {
            connection: "exercise",
            command: "launch_level",
            level: `${level}`,
          },
          "*"
        );
      }
      if (event.data.includes("Ping")) {
        if (!websockets_connected && event.data === "Ping3") {
          level = 4;
          console.log("HEEREEEE");
          connectionUpdate(
            {
              connection: "exercise",
              command: "launch_level",
              level: `${level}`,
            },
            "*"
          );
          websockets_connected = true;
          declare_code(address_code);
          declare_gui(address_gui);
        }
        if (gazeboToggle) {
          console.log("toggle gazebo");
          if (gazeboOn) {
            ws_manager.send(JSON.stringify({ command: "startgz" }));
          } else {
            ws_manager.send(JSON.stringify({ command: "stopgz" }));
          }
          gazeboToggle = false;
          // setGazeboToggle(false);
        } else if (simStop) {
          ws_manager.send(JSON.stringify({ command: "stop" }));
          simStop = false;
          running = false;
          // setRunning(false);
        } else if (simReset) {
          console.log("reset simulation");
          ws_manager.send(JSON.stringify({ command: "reset" }));
          simReset = false;
          // setSimReset(false);
        } else if (sendCode) {
          console.log("RUNNNNNN  --> HEEREEEE");

          let python_code = editorCode;
          python_code = "#code\n" + python_code;
          ws_manager.send(
            JSON.stringify({ command: "evaluate", code: python_code })
          );
          sendCode = false;
          // setSendCode(false);
        } else if (simResume) {
          ws_manager.send(JSON.stringify({ command: "resume" }));
          // setSimResume(false);
          simResume = false;
          running = true;
          // setRunning(true);
        } else {
          console.log("RUNNNNNN  --> NOthingggg");
          setTimeout(function () {
            ws_manager.send(JSON.stringify({ command: "Pong" }));
          }, 1000);
        }
      }
      if (event.data.includes("evaluate")) {
        if (event.data.length < 9) {
          // If there is an error it is sent along with "evaluate"
          console.log("EVENT CODE SENT --> APPY");
          submitCode();
        } else {
          let error = event.data.substring(10, event.data.length);
          connectionUpdate(
            { connection: "exercise", command: "error", text: error },
            "*"
          );
        }
        setTimeout(function () {
          ws_manager.send(JSON.stringify({ command: "Pong" }));
        }, 1000);
      } else if (event.data.includes("reset")) {
        // ResetEvaluator();
      } else if (event.data.includes("PingDone")) {
        enableSimControls();
        if (resetRequested === true) {
          resetRequested = false;
          // setResetRequested(false);
        }
      } else if (event.data.includes("style")) {
        let error = event.data.substring(5, event.data.length);
        connectionUpdate(
          { connection: "exercise", command: "style", text: error },
          "*"
        );
      }
    };
  };

  function resetSimulation() {
    simReset = true;
    // setSimReset(true);
  }

  function stopSimulation() {
    simStop = true;
  }

  function resumeSimulation() {
    simResume = true;
    // setSimResume(true);
  }

  function connectionUpdate(data) {
    console.log(data);
    if (data.connection === "manager") {
      if (data.command === "up") {
        setConnectionState("Connected");
        // launchButton.prop("disabled", false);
      } else if (data.command === "down") {
        setConnectionState("Connect");
        // connectionButton
        //   .removeClass("btn-success btn-warning")
        //   .addClass("btn-secondary");
        // connectionButton.innerHTML =
        //   '<span id="loading-connection" class="bi bi-arrow-down-up"></span> Connect';
        // connectionButton.prop("disabled", false);
        if (websocket_code != null) websocket_code.close();
        if (websocket_gui != null) websocket_gui.close();
        setLaunchState("Launch");
      }
    } else if (data.connection === "exercise") {
      if (data.command === "available") {
        // Nothing !
      } else if (data.command === "up") {
        stop();
        swapping = false;
        // setSwapping(false);
        setLaunchState("Ready");
        togglePlayPause(false);
        let reset_button = document.getElementById("reset");
        reset_button.disabled = false;
        reset_button.style.opacity = "1.0";
        reset_button.style.cursor = "default";
        let load_button = document.getElementById("loadIntoRobot");
        load_button.disabled = false;
        load_button.style.opacity = "1.0";
        load_button.style.cursor = "default";
      } else if (data.command === "down") {
        if (!swapping) {
          setLaunchState("Launch");
        }
      } else if (data.command === "swap") {
        setLaunchState("Launching");
      } else if (data.command === "launch_level") {
        var level = data.level;
        setLaunchLevel(level);
        setLaunchState("Launching");
      } else if (data.command === "error") {
        // $("#errorModal .modal-header .modal-header-text").text(
        //   "Errors detected:"
        // );
        // $("#errorModal .modal-body").text(data.text);
        // $("#errorModal").modal({ show: true, backdrop: false });
        // $("#errorModal .modal-dialog").draggable({});
        toggleSubmitButton(true);
      } else if (data.command === "style") {
        // $("#errorModal .modal-header .modal-header-text").text(
        //   "Style evaluation:"
        // );
        // if (data.text.replace(/\s/g, "").length)
        //   $("#errorModal .modal-body").text(data.text);
        // else $("#errorModal .modal-body").text("Everything is correct!");
        // $("#errorModal").modal({ show: true, backdrop: false });
        // $("#errorModal .modal-dialog").draggable({});
      }
    }
  }

  function enablePlayPause(enable) {
    let playPause_button = document.getElementById("submit");
    if (enable === false) {
      playPause_button.disabled = true;
      playPause_button.style.opacity = "0.4";
      playPause_button.style.cursor = "not-allowed";
    } else if (firstCodeSent === true) {
      playPause_button.disabled = false;
      playPause_button.style.opacity = "1.0";
      playPause_button.style.cursor = "default";
    }
  }

  const changeconsole = () => {
    setOpenConsole(!openConsole);
    setAlertState({
      ...alertState,
      errorAlert: false,
      successAlert: false,
      warningAlert: false,
      infoAlert: true,
    });
    setAlertContent(`Connection with Console Successful`);
  };

  const changegzweb = () => {
    gazeboOn = !gazeboOn;
    setOpenGazebo(gazeboOn);
    gazeboToggle = true;
    setAlertState({
      ...alertState,
      errorAlert: false,
      successAlert: false,
      warningAlert: false,
      infoAlert: true,
    });
    setAlertContent(`Connection with Gazebo Successful`);
  };

  const toggleSubmitButton = (toggle) => {
    var loadIntoRobot = document.getElementById("loadIntoRobot");
    if (toggle === false) {
      loadIntoRobot.disabled = true;
      loadIntoRobot.style.opacity = "0.4";
      loadIntoRobot.style.cursor = "not-allowed";
      handleLoadModalOpen();
    } else {
      loadIntoRobot.disabled = false;
      loadIntoRobot.style.opacity = "1.0";
      loadIntoRobot.style.cursor = "default";
      handleLoadModalClose();
    }
  };

  function toggleResetButton(toggle) {
    let reset_button = document.getElementById("reset");
    if (toggle === false) {
      reset_button.disabled = true;
      reset_button.style.opacity = "0.4";
      reset_button.style.cursor = "not-allowed";
    } else {
      reset_button.disabled = false;
      reset_button.style.opacity = "1.0";
      reset_button.style.cursor = "default";
    }
  }

  function togglePlayPause(stop) {
    setPlayState(!stop);
  }

  function enableSimControls() {
    if (resetRequested === true) {
      togglePlayPause(false);
    }
    enablePlayPause(true);
    toggleResetButton(true);
  }

  // Function to resume the simulation
  const start = () => {
    enablePlayPause(false);
    toggleResetButton(false);
    // Manager Websocket
    if (running === false) {
      resumeSimulation();
      //check(); // should be replaced by resumeBrain() when available
    }

    // GUI Websocket
    unpause_lap();

    // Toggle start/pause
    togglePlayPause(true);
  };

  function editorChanged(toggle) {
    if (firstCodeSent) {
      if (toggle) {
        // document.getElementById("loadIntoRobotAlert").style.display =
        //   "inline-block";
        // document.getElementById("loadIntoRobot").title =
        //   "Code changed since last sending";
      } else {
        // document.getElementById("loadIntoRobotAlert").style.display = "none";
        // document.getElementById("loadIntoRobot").title = "";
      }
    }
  }
  // Function to request to load the student code into the robot
  const check = () => {
    editorChanged(false);
    toggleSubmitButton(false);
    // setSendCode(true);
    sendCode = true;
  };

  // Function to stop the student solution
  const stop = () => {
    enablePlayPause(false);
    toggleResetButton(false);
    //stopCode(); // should be replaced by pauseBrain() when available
    // Manager Websocket
    if (running === true) {
      stopSimulation();
    }

    // GUI Websocket
    pause_lap();

    // Toggle start/pause
    togglePlayPause(false);
  };

  // Function to reset the simulation
  function resetSim() {
    resetRequested = true;
    // setResetRequested(true);
    toggleResetButton(false);
    enablePlayPause(false);

    // Manager Websocket
    resetSimulation();

    // GUI Websocket
    reset_gui();
    running = false;
    // setRunning(false);
  }
  const loadButtonClick = () => {};

  const teleopButtonClick = () => {
    if (!teleopMode) {
      if (!running) {
        resetSimulation();
        running = true;
        // setRunning(true);
      }
      setTeleopMode(true);
      document.addEventListener("keydown", keyHandler, false);
      document.addEventListener("keyup", keyHandler, false);
      return;
    }
    setTeleopMode(false);
    console.log("EVENT CODE SENT --> APPY FROM teleOp");
    submitCode();
    document.removeEventListener("keydown", keyHandler, false);
    document.removeEventListener("keyup", keyHandler, false);
  };

  function declare_code(websocket_address) {
    websocket_code = new WebSocket(websocket_address);

    websocket_code.onopen = function () {
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: "5" },
        "*"
      );
      if (websocket_gui.readyState === 1) {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: true,
          warningAlert: false,
          infoAlert: false,
        });
        setAlertContent(" Connection established! ");
        // alert("[open] Connection established!");
        connectionUpdate({ connection: "exercise", command: "up" }, "*");
      }
      websocket_code.send("#ping");
    };
    websocket_code.onclose = function (event) {
      connectionUpdate({ connection: "exercise", command: "down" }, "*");
      if (websocket_gui.readyState === 1) {
        if (event.wasClean) {
          alert(
            `[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`
          );
        } else {
          alert("[close] Connection closed!");
        }
      }
    };

    websocket_code.onmessage = function (event) {
      var source_code = event.data;
      let operation = source_code.substring(0, 5);

      if (operation === "#load") {
        setEditorCode(source_code.substring(5));
      } else if (operation === "#freq") {
        var frequency_message = JSON.parse(source_code.substring(5));
        // Parse GUI and Brain frequencies
        // document.querySelector("#ideal_gui_frequency").value =
        //   frequency_message.gui;
        console.log(frequency_message.gui);
        setGuiFreqValue(frequency_message.gui);
        // document.querySelector("#ideal_code_frequency").value =
        setCodeFreqValue(frequency_message.brain);
        console.log(frequency_message.brain);
        //   frequency_message.brain;
        // // Parse real time factor
        // document.querySelector("#real_time_factor").value =
        setRtfValue(frequency_message.rtf);

        // The acknowledgement messages invoke the python server to send further
        // messages to this client (inside the server's handle function)
        // Send the acknowledgment message along with frequency
        let code_frequency = 10;
        let gui_frequency = 10;
        let real_time_factor = 0.6;

        frequency_message = {
          brain: code_frequency,
          gui: gui_frequency,
          rtf: real_time_factor,
        };
        websocket_code.send("#freq" + JSON.stringify(frequency_message));
      } else if (operation === "#ping") {
        websocket_code.send("#ping");
      } else if (operation === "#exec") {
        if (firstCodeSent === false) {
          firstCodeSent = true;
          // setFirstCodeSent(true);
          enablePlayPause(true);
        }
        toggleSubmitButton(true);
      } else if (operation === "#stpd") {
        startNewCircuit();
      }

      // Send Teleop message if active
      if (teleopMode) {
        let teleop_message = { v: v, w: w };
        websocket_code.send("#tele" + JSON.stringify(teleop_message));
      }
    };
  }

  // To decode the image string we will receive from server
  function decode_utf8(s) {
    return decodeURIComponent(escape(s));
  }
  function declare_gui(websocket_address) {
    websocket_gui = new WebSocket(websocket_address);

    websocket_gui.onopen = function (event) {
      setLaunchLevel(launchLevel + 1);
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: "5" },
        "*"
      );
      if (websocket_code.readyState === 1) {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: true,
          warningAlert: false,
          infoAlert: false,
        });
        setAlertContent(" Connection established! ");
        // alert("[open] Connection established!");
        connectionUpdate({ connection: "exercise", command: "up" }, "*");
      }
    };

    websocket_gui.onclose = function (event) {
      connectionUpdate({ connection: "exercise", command: "down" }, "*");
      if (event.wasClean) {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: false,
          warningAlert: true,
          infoAlert: false,
        });
        setAlertContent(
          `Connection closed cleanly, code=${event.code} reason=${event.reason}`
        );
        // alert(
        //   `[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`
        // );
      } else {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: false,
          warningAlert: true,
          infoAlert: false,
        });
        setAlertContent(`Connection closed!`);
        // alert("[close] Connection closed!");
      }
    };

    // What to do when a message from server is received
    websocket_gui.onmessage = function (event) {
      let operation = event.data.substring(0, 4);
      let mapCanvas = document.getElementById("birds-eye");
      let canvas = document.getElementById("gui_canvas");
      if (operation === "#gui") {
        // Parse the entire Object
        let data = JSON.parse(event.data.substring(4));

        // Parse the Image Data
        (image_data = JSON.parse(data.image)),
          (source = decode_utf8(image_data.image)),
          (shape = image_data.shape);

        if (source != "" && running === true) {
          canvas.src = "data:image/jpeg;base64," + source;
        }
        // Parse the Map data
        // Slice off ( and )
        pose = data.map.substring(1, data.map.length - 1);
        content = pose.split(",").map(function (item) {
          return parseFloat(item);
        });
        setInitialPosition(content);
        drawCircle(content[0], content[1], content, mapCanvas);

        // Send the Acknowledgment Message
        websocket_gui.send("#ack");
      }
      // else if (operation === "#cor") {
      // 	// Set the value of command
      // 	command_input = event.data.substring(4,);
      // 	command.value = command_input;
      // 	// Go to next command line
      // 	next_command();
      // 	// Focus on the next line
      // 	command.focus();
      // }
    };
  }

  function pauseLap() {
    websocket_gui.send("#paus");
  }

  function unPauseLap() {
    websocket_gui.send("#resu");
  }

  function resetGui() {
    websocket_gui.send("#rest");
  }
  const editorCodeChange = (e) => {
    setEditorCode(e);
  };

  const onClickSave = () => {
    saveCode("testing", editorCode);
  };

  function startNewCircuit() {
    // Kill actual sim
    startSim(2);
    // StartSim
    swapping = true;
    // setSwapping(true);
    startSim(1, circuit);
    connectionUpdate({ connection: "exercise", command: "swap" }, "*");
    toggleSubmitButton(false);
    firstCodeSent = false;
    // setFirstCodeSent(false);
  }

  function handleCircuitChange(e, circuitSelector) {
    let circuit_ = e.target.value;
    let bImgSrc;
    circuitSelector.current.value = circuit_;
    setCircuit(circuit_);
    setBirdEyeClass(circuit_);
    // switch (circuit_) {
    //   case "montreal":
    //     bImgSrc = "/static/exercises/follow_line_react/img/montreal.jpg";
    //     setBackgroundImage(
    //       "/static/exercises/follow_line_react/img/montreal.jpg"
    //     );
    //     break;
    //   case "montmelo":
    //     bImgSrc = "/static/exercises/follow_line_react/img/montmelo.jpg";
    //     setBackgroundImage(
    //       "/static/exercises/follow_line_react/img/montmelo.jpg"
    //     );
    //     break;
    //   case "nbg":
    //     bImgSrc = "/static/exercises/follow_line_react/img/nbg.jpg";
    //     setBackgroundImage("/static/exercises/follow_line_react/img/nbg.jpg");
    //     break;
    //   default:
    //     bImgSrc = "/static/exercises/follow_line_react/img/map.jpg";
    //     setBackgroundImage("/static/exercises/follow_line_react/img/map.jpg");
    // }

    // let mapCanvas = document.getElementById("birds-eye");
    // let ctx = mapCanvas.getContext("2d");
    // ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    // let background = new Image();
    // background.src = bImgSrc;
    // // Make sure the image is loaded first otherwise nothing will draw.
    // background.onload = function () {
    //   scaleToFit(background, ctx, mapCanvas);
    // };
    // Disable connection button
    // connectionButton.current.prop("disabled", true);
    // Set variable to toggle gazebo
    // setGazeboToggle(true);
    gazeboToggle = true;
    // Stop the simulation
    stop();
    // Kill actual sim
    startSim(2);
    // // StartSim
    startSim(1, e.target.value);
    setAlertState({
      ...alertState,
      errorAlert: false,
      successAlert: false,
      warningAlert: false,
      infoAlert: true,
    });
    setAlertContent(
      `Loading circuit. Please wait until the connection is restored.`
    );
    // alert("Loading circuit. Please wait until the connection is restored.");
    connectionUpdate({ connection: "exercise", command: "down" }, "*");
  }

  function scaleToFit(img, ctx, canvas) {
    // get the scale
    const scale = Math.min(
      canvas.width / img.width,
      canvas.height / img.height
    );
    // get the top left position of the image
    const x = canvas.width / 2 - (img.width / 2) * scale;
    const y = canvas.height / 2 - (img.height / 2) * scale;
    ctx.drawImage(img, x, y, img.width * scale, img.height * scale);
  }

  const connectionButtonClick = () => {
    if (connectionState === "Connect") {
      setConnectionState("Connecting");
      startSim(0, "default");
    }
  };

  const launchButtonClick = () => {
    if (connectionState === "Connected" && launchState === "Launch") {
      setLaunchState("Launching");
      startSim(1, circuit);
    } else if (connectionState === "Connect") {
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: true,
        infoAlert: false,
      });
      setAlertContent(
        `A connection with the manager must be established before launching an exercise`
      );
      // alert(
      //   "A connection with the manager must be established before launching an exercise"
      // );
    }
  };

  function deactivateTeleopButton() {
    setTeleopMode(false);
    document.removeEventListener("keydown", keyHandler, false);
    document.removeEventListener("keyup", keyHandler, false);
  }

  const resumeBrain = () => {
    let message = "#play\n";
    console.log("Message sent!");
    websocket_code.send(message);
  };

  const stopBrain = () => {
    let message = "#stop\n";
    console.log("Message sent!");
    websocket_code.send(message);
  };

  const resetBrain = () => {
    let message = "#rest\n";
    console.log("Message sent!");
    websocket_code.send(message);
  };

  // Function that sends/submits the code!
  const submitCode = () => {
    try {
      // Get the code from editor and add headers
      var python_code = editorCode;
      python_code = "#code\n" + python_code;

      websocket_code.send(python_code);
      console.log("Code Sent! Check terminal for more information!");

      deactivateTeleopButton();
    } catch {
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: true,
        infoAlert: false,
      });
      setAlertContent(
        `Connection must be established before sending the code.`
      );
      // alert("Connection must be established before sending the code.");
    }
  };

  // Function that send/submits an empty string
  const stopCode = () => {
    var stop_code = "#code\n";
    console.log("Message sent!");
    websocket_code.send(stop_code);
  };

  // Function for range slider
  const codefrequencyUpdate = (vol) => {
    document.querySelector("#code_freq").value = vol;
  };

  // Function for range slider
  const guifrequencyUpdate = (vol) => {
    document.querySelector("#gui_freq").value = vol;
  };
  function keyHandler(event) {
    // Right (39), Left (37), Down (40), Up (38)

    // First check if websocket_gui_guest and websocket_code_guest are defined
    if (
      typeof websocket_gui !== "undefined" &&
      typeof websocket_code !== "undefined"
    ) {
      let cmd = "#tele";

      // Prevent using arrow keys to scroll page
      if ([32, 37, 38, 39, 40].indexOf(event.keyCode) > -1) {
        event.preventDefault();
      }

      if (event.keyCode === 39) {
        //console.log('Right');
        w = event.type === "keydown" ? -1 : 0;
      } else if (event.keyCode === 37) {
        //console.log('Left');
        w = event.type === "keydown" ? 1 : 0;
      } else if (event.keyCode === 40) {
        //console.log('Down');
        v = event.type === "keydown" ? -2 : 0;
      } else if (event.keyCode === 38) {
        //console.log('Up');
        v = event.type === "keydown" ? 2 : 0;
      }
      console.log("v: ", v, "w: ", w);
    }
  }

  function pause_lap() {
    websocket_gui.send("#paus");
  }

  function unpause_lap() {
    websocket_gui.send("#resu");
  }

  function reset_gui() {
    websocket_gui.send("#rest");
  }
  const loadFileButton = () => {
    var fr = new FileReader();
    fr.onload = function () {
      setEditorCode(fr.result, 1);
    };
    // fr.readAsText()
  };

  const [openLoadModal, setOpenLoadModal] = React.useState(false);
  const handleLoadModalOpen = () => setOpenLoadModal(true);
  const handleLoadModalClose = () => setOpenLoadModal(false);

  return (
    <ExerciseContext.Provider
      value={{
        editorCode,
        openLoadModal,
        handleLoadModalOpen,
        handleLoadModalClose,
        check,
        onClickSave,
        editorCodeChange,
        connectionState,
        launchState,
        firstCodeSent,
        connectionButtonClick,
        launchButtonClick,
        resetSim,
        start,
        stop,
        loadFileButton,
        startSim,
        circuit,
        // backgroundImage,
        scaleToFit,
        handleCircuitChange,
        getCircuitValue,
        launchLevel,
        loadButtonClick,
        teleopButtonClick,
        connectionUpdate,
        changegzweb,
        changeconsole,
        guiFreqValue,
        codeFreqValue,
        rtfValue,
        openGazebo,
        playState,
        birdEyeClass,
        openConsole,
        alertState,
        alertContent,
      }}
    >
      {children}
    </ExerciseContext.Provider>
  );
}

ExerciseProvider.propTypes = {
  children: PropTypes.node,
};

export default ExerciseContext;

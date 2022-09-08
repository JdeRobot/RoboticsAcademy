"use strict";
var circuit;
// Get the select element
const circuit_selector = document.querySelector("#circuit_selector");
circuit_selector.addEventListener("change", function () {
  // Get the value selected
  circuit = circuit_selector.options[circuit_selector.selectedIndex].value;
  console.log("Changed: ", circuit);
  // Classes
  let classes = ["default", "montreal", "montmelo", "nbg"];

  // Disable connection button
  //$("#connection-button").prop('disabled', true);
  // Set birds-eye background
  let canvas = document.querySelector("#birds-eye");
  classes.forEach((c) => {
    canvas.classList.remove(c);
  });
  canvas.classList.add(circuit);

  // Set variable to toggle gazebo
  gazeboToggle = true;
  // Stop the simulation
  stop();
  stopBrain();
  alert("Loading circuit. Please wait until the connection is restored.");
  connectionUpdate({ connection: "exercise", command: "down" }, "*");
});
function starNewCircuit() {
  // Kill actual sim
  startSim(2);
  // StartSim
  swapping = true;
  startSim(1, circuit);
  connectionUpdate({ connection: "exercise", command: "swap" }, "*");
  toggleSubmitButton(false);
  firstCodeSent = false;
}

// Main message controller for websockets
var running = true;
// Var to indicate whether a reset was requested
var resetRequested = false;
var firstCodeSent = false;

// Function to resume the simulation
function start(){
    enablePlayPause(false);
    toggleResetButton(false);
    // Manager Websocket
    if (running == false) {
        resumeSimulation();
        resumeBrain();
    }
    togglePlayPause(true);    
}

// Function to request to load the student code into the robot
function check() {
    editorChanged(false);
    toggleSubmitButton(false);
    checkCode();

}

// Function to stop the student solution
function stop(){
    enablePlayPause(false);
    toggleResetButton(false);

    // Manager Websocket
    if (running == true) {
        stopSimulation();
        stopBrain();
    }

    togglePlayPause(false);
}

// Function to reset the simulation
function resetSim(){
    resetRequested = true;
    toggleResetButton(false);
    toggleSubmitButton(false);
    enablePlayPause(false);

    // Manager Websocket
    resetBrain();
    resetSimulation();

    running = false;
}
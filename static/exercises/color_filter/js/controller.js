// Main message controller for websockets
var running = true;
// Var to indicate whether a reset was requested
var resetRequested = false;
var firstCodeSent = false;

// Function to resume the simulation
function start(){
    // Manager Websocket
    editorChanged(false);
    checkCode();
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
    //stopCode(); // should be replaced by pauseBrain() when available
    stopCode();

    togglePlayPause(false);
}

// Function to reset the simulation
function resetSim(){
    resetRequested = true;

    // Manager Websocket
    resetBrain();
    resetSimulation();

    running = false;
}
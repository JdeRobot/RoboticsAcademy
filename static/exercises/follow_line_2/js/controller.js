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
        //check(); // should be replaced by resumeBrain() when available
    }

    // GUI Websocket
    unpause_lap();

    // Toggle start/pause
    togglePlayPause(true);    
}

// Function to request to load the student code into the robot
function check(){
    editorChanged(false);
    toggleSubmitButton(false);
    checkCode();
}

// Function to stop the student solution
function stop(){
    enablePlayPause(false);
    toggleResetButton(false);
    //stopCode(); // should be replaced by pauseBrain() when available
    // Manager Websocket
    if (running == true) {
        stopSimulation();
    }

    // GUI Websocket
    pause_lap();

    // Toggle start/pause
    togglePlayPause(false);
}

// Function to reset the simulation
function resetSim(){
    resetRequested = true;
    toggleResetButton(false);
    enablePlayPause(false);

    // Manager Websocket
    resetSimulation();

    // GUI Websocket
    reset_gui();

    running = false;
}
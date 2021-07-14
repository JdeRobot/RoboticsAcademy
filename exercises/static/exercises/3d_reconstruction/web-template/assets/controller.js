// Main message controller for websockets
var running = true;

// Function to execute the student solution
function start(){
    // Code Websocket
    submitCode();

    // GUI Websocket
    unpause_lap();

    // Mark the variables
    running = true;
}

// Function to stop the student solution
function stop(){
    // Code Websocket
    stopCode();

    // GUI Websocket
    pause_lap();

    // Mark the variables
    running = false;
}

// Function to reset the simulation
function reset(){
    // Code websocket
    stopCode();

    // Manager Websocket
    resetSimulation();

    // GUI Websocket
    reset_gui();

    // Code Websocket
    if (running == true) {
        submitCode();
    }
}
// Main message controller for websockets
var running = true;

// Function to execute the student solution
function start(){
    // Code Websocket
    submitCode();

    // Mark the variables
    running = true;
}

// Function to stop the student solution
function stop(){
    // Code Websocket
    stopCode();

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
    clearMap();

    // Code Websocket
    if (running == true) {
        submitCode();
    }
}
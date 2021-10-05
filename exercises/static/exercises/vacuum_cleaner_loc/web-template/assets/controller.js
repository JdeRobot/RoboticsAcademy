// Main message controller for websockets
var running = false;

// Function to execute the student solution
function start(){
    // Manager Websocket
    if (running == false) {
        resumeSimulation();
    }

    // Code Websocket
    submitCode();
    reset_evaluator_map();
}

function check() {
    checkCode();
}

// Function to stop the student solution
function stop(){
    // Code Websocket
    stopCode();

    // Manager Websocket
    if (running == true) {
        stopSimulation();
    }
}

// Function to reset the simulation
function reset(){
    // Code Websocket
    stopCode();

    // Manager Websocket
    resetSimulation();

    // Canvas
    reset_evaluator_map();

    // Code Websocket
    if (running == true) {
        submitCode();
    }
}
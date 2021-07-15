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
}

// Function to stop the student solution
function stop(){
    // Manager Websocket
    if (running == true) {
        stopSimulation();
    }
}

// Function to reset the simulation
function reset(){
    // Manager Websocket
    resetSimulation();

    // Reset GUI
    reset_scene3d();
    reset_matching();

    // Code Websocket
    if (running == true) {
        submitCode();
    }
}
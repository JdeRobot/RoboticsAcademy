var ws_manager;
var gazeboToggle = false, gazeboOn = false;
var simReset = false;
var simStop = false;
var simResume = false;
var sendCode = false;

function startSim(step) {
    var level = 0;
    let websockets_connected = false;
    if (step == 0) {
        ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    }
    else if (step == 1) {
        radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        var size = get_novnc_size();
        ws_manager.send(JSON.stringify({
            "command": "open", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString()}));
        level++;
        radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        ws_manager.send(JSON.stringify({"command" : "Pong"}));
    }
    else if (step == 2) {
        ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
    }

    ws_manager.onopen = function (event) {
        level++;
        radiConect.contentWindow.postMessage({connection: 'manager', command: 'up'}, '*');
        radiConect.contentWindow.postMessage({connection: 'exercise', command: 'available'}, '*');
    }

    ws_manager.onmessage = function (event) {
        //console.log(event.data);
        if (event.data.level > level) {
            level = event.data.level;
            radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        }
        if (event.data.includes("Ping")) {
            if (!websockets_connected && event.data == "Ping3") {
                level = 4;
                radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
                websockets_connected = true;
                declare_code(websocket_address);
                declare_gui(websocket_address);
            }
            if (gazeboToggle) {
                console.log("toggle gazebo");
                if (gazeboOn) {
                    ws_manager.send(JSON.stringify({"command" : "startgz"}));
                } else {
                    ws_manager.send(JSON.stringify({"command" : "stopgz"}));
                }

                gazeboToggle = false;
            } else if (sendCode) {
                let python_code = editor.getValue();
		        python_code = "#code\n" + python_code;
                ws_manager.send(JSON.stringify({"command": "evaluate", "code": python_code}));
                sendCode = false;
            }else if (simReset){
                console.log("reset simulation");
                ws_manager.send(JSON.stringify({"command": "reset"}));
                simReset = false;
            } else if (simStop){
                ws_manager.send(JSON.stringify({"command": "stop"}));
                simStop = false;
            } else if (simResume){
                ws_manager.send(JSON.stringify({"command": "resume"}));
                simResume = false;
            } else {
                setTimeout(function () {
                    ws_manager.send(JSON.stringify({"command" : "Pong"}));
                }, 1000)
            }
        }
        if (event.data.includes("evaluate")) {
                if (event.data.length < 9) {    // If there is an error it is sent along with "evaluate"
                    submitCode();
                } else {                
                    let error = event.data.substring(10,event.data.length);
                    radiConect.contentWindow.postMessage({connection: 'exercise', command: 'error', text: error}, '*');
                    toggleSubmitButton(true);
                }
                setTimeout(function () {
                    ws_manager.send(JSON.stringify({"command" : "Pong"}));
                }, 1000)
            } else if (event.data.includes("PingDone")) {
                enablePlayPause(true);    
                toggleResetButton(true);       
                if (resetRequested == true) {
                    togglePlayPause(false);
                    resetRequested = false;
            }
        }
    }
}

function toggleGazebo() {
    if (gazeboOn) {
        gazeboOn = false;
    } else {
        gazeboOn = true;
    }

    gazeboToggle = true;
}

function resetSimulation() {
    simReset = true;
}

function stopSimulation() {
    simStop = true;
}

function resumeSimulation() {
    simResume = true;
}

function checkCode() {
    sendCode = true;
}
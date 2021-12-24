var ws_manager;
var gazeboToggle = false, gazeboOn = false;
var simReset = false;
var simStop = false;
var simResume = false;
var sendCode = false;
var address_code;
var address_gui;
var first_attempt = true;

function startSim(step){
    var level = 0;
    let websockets_connected = false;

    if (step == 0) {
        $("#connection-button").removeClass("btn-danger").addClass("btn-warning");
		$("#connection-button").html('<span id="loading-connection" class="fa fa-refresh fa-spin"></span> Connecting');
        address_code = "ws://" + websocket_address + ":1905";
        address_gui = "ws://" + websocket_address + ":2303";
        ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    }
    else if (step == 1) {
        connectionUpdate({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        var size = get_novnc_size();
        ws_manager.send(JSON.stringify({
            "command": "open", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString()}));
        level++;
        connectionUpdate({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        ws_manager.send(JSON.stringify({"command" : "Pong"}));
    }
    else if (step == 2) {
        ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
    }

    ws_manager.onopen = function (event) {
        level++;
        connectionUpdate({connection: 'manager', command: 'up'}, '*');
        connectionUpdate({connection: 'exercise', command: 'available'}, '*');
    }

    ws_manager.onclose = function (event) {
        connectionUpdate({connection: 'manager', command: 'down'}, '*');
        if (!first_attempt) {
            alert("Connection lost, retrying connection...");
            startSim(step, websocket_address, server, username);
        } else {
            first_attempt = false;
        }
    }

    ws_manager.onerror = function (event) {
        connectionUpdate({connection: 'manager', command: 'down'}, '*');
    }

    ws_manager.onmessage = function (event) {
        //console.log(event.data);
        if (event.data.level > level) {
            level = event.data.level;
            connectionUpdate({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        }
        if (event.data.includes("Ping")) {
            if (!websockets_connected && event.data == "Ping3") {
                level = 4;
                connectionUpdate({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
                websockets_connected = true;
                declare_code(address_code);
                declare_gui(address_gui);
            }
            if (gazeboToggle) {
                if (gazeboOn) {
                    ws_manager.send(JSON.stringify({"command" : "startgz"}));
                } else {
                    ws_manager.send(JSON.stringify({"command" : "stopgz"}));
                }

                gazeboToggle = false;
            } else if (simStop){
                ws_manager.send(JSON.stringify({"command": "stop"}));
                simStop = false;
                running = false;
            }else if (simReset){
                ws_manager.send(JSON.stringify({"command": "reset"}));
                simReset = false;
            }else if (sendCode) {
                let python_code = editor.getValue();
		        python_code = "#code\n" + python_code;
                ws_manager.send(JSON.stringify({"command": "evaluate", "code": python_code}));
                sendCode = false;
            } else if (simResume){
                ws_manager.send(JSON.stringify({"command": "resume"}));
                simResume = false;
                running = true;
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
                connectionUpdate({connection: 'exercise', command: 'error', text: error}, '*');
            }
            setTimeout(function () {
                ws_manager.send(JSON.stringify({"command" : "Pong"}));
            }, 1000)
        } else if (event.data.includes("reset")) {
            clearMap();
            reset_evaluator_map();
        } else if (event.data.includes("PingDone")) {
            enableSimControls();
            if (resetRequested == true) {
                resetRequested = false;
            }
        }
        else if (event.data.includes("style")) {
            let error = event.data.substring(5, event.data.length);
            connectionUpdate({connection: 'exercise', command: 'style', text: error}, '*');
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
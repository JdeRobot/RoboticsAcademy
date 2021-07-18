var ws_manager;

function startSim(step) {
    exercise = "human_detection";
    var level = 0;
    let websockets_connected = false;
    if (step == 0) {
        ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    }
    else if (step == 1) {
        radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        ws_manager.send(JSON.stringify({
            "command": "open", "exercise": exercise}));
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
            setTimeout(function () {
                ws_manager.send(JSON.stringify({"command" : "Pong"}));
            }, 1000)
        }        
    }
}
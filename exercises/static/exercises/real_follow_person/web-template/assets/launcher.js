function startSim() {
    ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    exercise = "real_follow_person"
    var level = 0;
    let websockets_connected = false;

    ws_manager.onopen = function (event) {
        level++;
        radiConect.contentWindow.postMessage({command: 'launch_level', level: `${level}`}, '*');
        ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
        ws_manager.send(JSON.stringify({
            "command": "open", "exercise": exercise}));
        level++;
        radiConect.contentWindow.postMessage({command: 'launch_level', level: `${level}`}, '*');
        ws_manager.send(JSON.stringify({"command" : "Pong"}));
    }

    ws_manager.onmessage = function (event) {
        //console.log(event.data);
        if (event.data.level > level) {
            level = event.data.level;
            radiConect.contentWindow.postMessage({command: 'launch_level', level: `${level}`}, '*');
        }
        if (event.data.includes("Ping")) {
            if (!websockets_connected && event.data == "Ping3") {
                level = 4;
                radiConect.contentWindow.postMessage({command: 'launch_level', level: `${level}`}, '*');
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

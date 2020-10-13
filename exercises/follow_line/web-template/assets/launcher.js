ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
exercise = "follow_line"

ws_manager.onopen = function(event){
    ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
}

ws_manager.send(JSON.stringify({"command": "open", "exercise": exercise}));

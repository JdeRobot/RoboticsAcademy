ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
exercise = "follow_line"

ws_manager.onopen = function(event){
    ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
    ws_manager.send(JSON.stringify({"command": "open", "exercise": exercise}));
}

//We must receive 2 "Done" messages before we can connect/disconnect
var rcv_msgs = 0;
ws_manager.onmessage = function(event){
    console.log(event.data);
    rcv_msgs += 1;
    if (rcv_msgs==2){
        setup_completed=true;
    }
}
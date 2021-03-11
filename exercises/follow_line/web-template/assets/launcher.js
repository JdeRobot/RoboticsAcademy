ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
exercise = "follow_line"

ws_manager.onopen = function(event){
    var width = window.innerWidth || document.documentElement.clientWidth || document.body.clientWidth;
    var height = window.innerHeight || document.documentElement.clientHeight || document.body.clientHeight; 
    ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
    ws_manager.send(JSON.stringify({"command": "open", "exercise": exercise,
        "width": width.toString(), "height": height.toString()}));
}

ws_manager.onmessage = function(event){
    console.log(event.data);
}

setTimeout(function(){
    declare_code(websocket_address);
    declare_gui(websocket_address);
}, 20000);
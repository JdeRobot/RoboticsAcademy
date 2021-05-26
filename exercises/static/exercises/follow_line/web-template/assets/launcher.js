function startSim() {
    ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    exercise = "follow_line"

    ws_manager.onopen = function (event) {
        var size = get_novnc_size();
        ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
        ws_manager.send(JSON.stringify({
            "command": "open", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString()}));
    }

    ws_manager.onmessage = function (event) {
        console.log(event.data);
        ws_manager.send("Pong");
    }

    setTimeout(function () {
        declare_code(websocket_address);
        declare_gui(websocket_address);
    }, 20000);
}
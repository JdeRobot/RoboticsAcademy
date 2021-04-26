function startSim() {
    var dev_mode = document.getElementById("dev_mode_checkbox");
    if (!dev_mode.checked) {
        console.log("user-mode")
        ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
        exercise = "follow_road"

        ws_manager.onopen = function (event) {
            var size = get_novnc_size();
            ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
            ws_manager.send(JSON.stringify({"command": "open", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString()}));
        }

        ws_manager.onmessage = function (event) {
            console.log(event.data);
        }
    } else {
        console.log("dev-mode")
    }

    setTimeout(function () {
        declare_code(websocket_address);
        declare_gui(websocket_address);
    }, 20000);
}

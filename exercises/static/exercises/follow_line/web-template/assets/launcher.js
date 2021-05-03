var ws_manager, exercise;
var callbacks = {};

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
        if (typeof event.data === "string" && event.data.startsWith("Done") && event.data.split("#")[1]) {
            var operation = event.data.split("#")[1]
            receivedAck(operation);
        }
    }

    setTimeout(function () {
        declare_code(websocket_address);
        declare_gui(websocket_address);
    }, 20000);
}

function sendMessage(name, data, callback) {
    callbacks[name] = callback;
    ws_manager.send(data);
}

function receivedAck(name) {
    if (name in callbacks) {
        callbacks[name]();
        delete callbacks[name];
    }
}

function startGzclient() {
    var size = get_novnc_size();
    sendMessage("start-gzclient", JSON.stringify({
        "command": "start-gzclient", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString()}),
        function(){
            var iframe = document.getElementById("iframeGzweb");
            iframe.setAttribute("src", "http://" + websocket_address + ":6080/vnc.html?resize=remote&autoconnect=true");
            iframe.style.display = 'block';
        });
}

function stopGzclient() {
    sendMessage("stop-gzclient", JSON.stringify({
        "command": "stop-gzclient"}), function() {
            document.getElementById('iframeGzweb').style.display = 'none';
        });
}

function startConsole() {
    var size = get_novnc_size();
    sendMessage("start-console", JSON.stringify({
        "command": "start-console", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString()}),
        function(){
            var iframe = document.getElementById("console");
            iframe.setAttribute("src", "http://" + websocket_address + ":1108/vnc.html?resize=remote&autoconnect=true");
            iframe.style.display = 'block';
        });
}

function stopConsole() {
    sendMessage("stop-console", JSON.stringify({
        "command": "stop-console"}), function() {
            document.getElementById('console').style.display = 'none';
        });
}

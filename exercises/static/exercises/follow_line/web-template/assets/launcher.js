function startSim() {
    ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    exercise = "follow_line"
    var level = 1;

    ws_manager.onopen = function (event) {
        var size = get_novnc_size();
        level = 2;
        ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
        ws_manager.send(JSON.stringify({
            "command": "open", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString()}));
        document.getElementById("launch_level").innerHTML = 2;
    }

    ws_manager.onmessage = function (event) {
        // console.log(event.data);
        
        //Aquí debería leer si cuando recibe el mensaje tiene x comando
        //Podrías hacer que siempre lea event.data para sacar el valor del número independientemente del mensaje?
        ws_manager.send(JSON.stringify({"command" : "Pong"}));
        //Así podrías conseguir que level se actualize correctamente y por lo tanto actualizar launch_level que es el elemento que contiene o modifica el número
        //Si event.data llevase otro valor en otro mensaje, tendras que sustituirlo por una variable que envies en el websocket
        
        if (event.data < 5) //Así nos aseguramos que en un futuro donde haya avanzado más pasos pero siga enviando mensajes pues no vuelva a actualizarlo
            level = event.data;
            document.getElementById("launch_level").innerHTML = level;
        
    }

    //Esto supongo que habría que eliminarlo y recolocarlo cuando haya llegado el paso correspondiente
    setTimeout(function () {
        declare_code(websocket_address);
        declare_gui(websocket_address);
    }, 20000);
}

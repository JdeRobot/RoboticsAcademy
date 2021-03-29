ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
exercise = "follow_line"

function get_inner_size(element) {
    var cs = getComputedStyle(element);
    var padding_x = parseFloat(cs.paddingLeft) + parseFloat(cs.paddingRight);
    var padding_y = parseFloat(cs.paddingTop) + parseFloat(cs.paddingBottom);

    var border_x = parseFloat(cs.borderLeftWidth) + parseFloat(cs.borderRightWidth);
    var border_y = parseFloat(cs.borderTopWidth) + parseFloat(cs.borderBottomWidth);

    // Element width and height minus padding and border
    width = element.offsetWidth - padding_x - border_x;
    height = element.offsetHeight - padding_y - border_y;

    return {width: Math.floor(width), height: Math.floor(height)};
}

// ws_manager.onopen = function(event){
//     var inner_size = get_inner_size(document.querySelector(".split.b"));
//     var width = inner_size.width || document.body.clientWidth;
//     // Since only 50% of height is used for gazebo iframe
//     var height = Math.floor(0.5 * inner_size.height) || document.body.clientHeight;
//     ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
//     ws_manager.send(JSON.stringify({"command": "open", "exercise": exercise,
//         "width": width.toString(), "height": height.toString()}));
// }

// ws_manager.onmessage = function(event){
//     console.log(event.data);
// }

// setTimeout(function(){
//     declare_code(websocket_address);
//     declare_gui(websocket_address);
// }, 20000);



var btn_state = 0;

async function ws_connect(){
document.getElementById('connect').disabled = true;
ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
ws_manager.onopen = async function(event){
    var inner_size = get_inner_size(document.querySelector(".split.b"));
    var width = inner_size.width || document.body.clientWidth;
    // Since only 50% of height is used for gazebo iframe
    var height = Math.floor(0.5 * inner_size.height) || document.body.clientHeight;
    // await ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
    await ws_manager.send(JSON.stringify({"command": "open", "exercise": exercise,"width": width.toString(), "height": height.toString()}));
}
ws_manager.onmessage = async function(event){
    console.log(event.data);
    if(event.data === 'Done'){
        try{
        await declare_code(websocket_address);
        await declare_gui(websocket_address);
        }
        catch(err){console.log(err)}
    }   
}
}

async function ws_disconnect(){
   ws_disconnect_code()
   ws_disconnect_gui()
   document.getElementById('connect').innerText = 'connect';

}

async function connect_btn(){
    console.log(btn_state)
    if(btn_state == 0 ){
       await ws_connect();
    }
    else if(btn_state == 2){
       await ws_disconnect();
    }

}
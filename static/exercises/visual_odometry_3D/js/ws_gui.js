// To decode the image string we will receive from server
function decode_utf8(s) {
    return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui, animation_id;
var canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');
var image = new Image();
image.src = "/static/exercises/assets/kitti/dataset/sequences/01/image_0/000000.png";

function declare_gui(websocket_address) {
    websocket_gui = new WebSocket(websocket_address);
  
    websocket_gui.onopen = function (event) {
        //alert("[open] Connection established!");
        set_launch_level(get_launch_level() + 1);
        if (websocket_code.readyState == 1) {
            alert("[open] Connection established!");
            connectionUpdate({ connection: 'exercise', command: 'up' }, '*');
        }
    }

  
    websocket_gui.onclose = function (event) {
        connectionUpdate({ connection: 'exercise', command: 'down' }, '*');
        if (event.wasClean) {
            //alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
        } else {
            //alert("[close] Connection closed!");
        }
    }
  
    // What to do when a message from server is received
    websocket_gui.onmessage = function (event) {
        operation = event.data.substring(0, 4);
    
        if (operation == "#gui") {
            // Parse the entire Object
            var data = JSON.parse(event.data.substring(4));
    
            // Parse the Image Data
            var image_data = JSON.parse(data.image),
                source = decode_utf8(image_data.image),
                shape = image_data.shape,
                counter = image_data.counter,
                true_euler_angles = image_data.true_euler_angles,
                true_position = image_data.true_position,
                estimated_euler_angles = image_data.estimated_euler_angles,
                estimated_position = image_data.estimated_position;
    
                console.log("Counter: ", counter);
                // console.log(true_euler_angles);
                // console.log(true_position);
                // console.log(estimated_euler_angles);
                // console.log(estimated_position);

            if (source != "") {
                image.src = "data:image/jpeg;base64," + source;
                canvas.width = shape[1];
                canvas.height = shape[0];
            }
    
            // Send the Acknowledgment Message
            websocket_gui.send("#ack");
        }
    }
  }

// For image object
image.onload = function () {
    update_image();
}

// Request Animation Frame to remove the flickers
function update_image() {
    animation_id = window.requestAnimationFrame(update_image);
    context.drawImage(image, 0, 0);
}

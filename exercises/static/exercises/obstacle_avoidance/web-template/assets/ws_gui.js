// Set the src of iframe tag
//document.getElementById("gzweb").setAttribute(
//	"src", "http://" + websocket_address + ":8080")

// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui, operation, data;
var image_data, source, shape;
var lap_time, map_data;
var command_input;

function declare_gui(websocket_address){
	websocket_gui = new WebSocket("ws://" + websocket_address + ":2303/");

	websocket_gui.onopen = function(event){
		if (!resetRequested)
			radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: '6'}, '*');
		if (websocket_code.readyState == 1) {
			if (!resetRequested) {
                alert("[open] Connection established!");
                radiConect.contentWindow.postMessage({connection: 'exercise', command: 'up'}, '*');
            }
            enableSimControls();
		}
	}

	websocket_gui.onclose = function(event){
		// Check for hard reset (reboot exercise.py)
		if (resetRequested && ws_manager.readyState == 1) {
			setTimeout(function () {
				declare_gui(websocket_address);
			}, 500)
		}
		else {
			radiConect.contentWindow.postMessage({connection: 'exercise', command: 'down'}, '*');
			if(event.wasClean){
				alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
			}
			else{
				alert("[close] Connection closed!");
			}
		}
	}

	// What to do when a message from server is received
	websocket_gui.onmessage = function(event){
		operation = event.data.substring(0, 4);
		if(operation == "#gui"){
			// Parse the entire Object
			data = JSON.parse(event.data.substring(4, ));

			// Parse the image data
			image_data = JSON.parse(data.image);
			source = decode_utf8(image_data.image);
			shape = image_data.shape;

			if(source != ""){
				canvas.src = "data:image/jpeg;base64," + source;
				canvas.width = shape[1];
				canvas.height = shape[0];
			}

			// Parse the Lap data
			lap_time = data.lap;
			if(lap_time != ""){
				lap_time_display.textContent = lap_time;
			}

			// Parse the Map data
			map_data = JSON.parse(data.map);
			paintEvent(map_data.target, map_data.car, map_data.obstacle, map_data.average, map_data.laser, map_data.max_range);

			// Send the Acknowledgement Message
			websocket_gui.send("#ack");
		}
	
	};
}

var canvas = document.getElementById("gui_canvas");
    
// Lap time DOM
var lap_time_display = document.getElementById("lap_time");

// Set the src of iframe tag
//document.getElementById("gzweb").setAttribute(
//	"src", "http://" + websocket_address + ":8080")

// To decode the image string we will receive from server
function decode_utf8(s) {
	return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui, animation_id;
var image_data, source, shape;
var lap_time, pose, content;
var command_input;

function declare_gui(websocket_address) {
	websocket_gui = new WebSocket(websocket_address);
	let canvas = document.getElementById("gui-canvas-numpy");

	websocket_gui.onopen = function (event) {
		//alert("[open] Connection established!");
		set_launch_level(get_launch_level()+1);
		connectionUpdate({connection: 'exercise', command: 'launch_level', level: '6'}, '*');
		if (websocket_code.readyState == 1) {
			alert("[open] Connection established!");
			connectionUpdate({connection: 'exercise', command: 'up'}, '*');
		}
	}

	websocket_gui.onclose = function (event) {
		connectionUpdate({connection: 'exercise', command: 'down'}, '*');	

		if (event.wasClean) {
			//alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
		}
		else {
			//alert("[close] Connection closed!");
		}
	}

	// What to do when a message from server is received
	websocket_gui.onmessage = function (event) {
		operation = event.data.substring(0, 4);
	
		if (operation == "#gui") {
			// Parse the entire Object
			var data = JSON.parse(event.data.substring(4, ));
			//Parse array data and call generathPath function		
			array = JSON.parse(data.array);
			generatePath(array);

			//Parse encoded image data and decode it
			var image_data = JSON.parse(data.image),
			source = decode_utf8(image_data.image),
			shape = image_data.shape;

			if(source !== ""){
				canvas.src = "data:image/png;base64," + source;
				canvas.width = shape[1];
				canvas.height = shape[0];
			}

			// Parse the Map data
			// Slice off ( and )
			pose = data.map.substring(1, data.map.length - 1);
			content = pose.split(',').map(function(item) {
				return parseFloat(item);
			})
			draw(content[0], content[1], content[2], content[3]);

			// Send the Acknowledgment Message
			websocket_gui.send("#ack");
		}

		else if (operation == "#cor") {
			// Set the value of command
			command_input = event.data.substring(4,);
			command.value = command_input;
			// Go to next command line
			next_command();
			// Focus on the next line
			command.focus();
		}
	}
}

//function to send the mouse click data
function pickLoc(){
        var data = destinationPicker(event);
        coords = {"data" : data};
        websocket_gui.send("#pick" + JSON.stringify(coords));
}

function reset_gui(){
	websocket_gui.send("#rest");
}
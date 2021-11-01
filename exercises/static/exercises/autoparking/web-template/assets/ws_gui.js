// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui, operation, data;
var map_data;

function declare_gui(websocket_address){
	websocket_gui = new WebSocket("ws://" + websocket_address + ":2303/");

	websocket_gui.onopen = function(event){
		radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: '6'}, '*');
		if (websocket_code.readyState == 1) {
			alert("[open] Connection established!");
			radiConect.contentWindow.postMessage({connection: 'exercise', command: 'up'}, '*');
			enableSimControls();
		}
	}
	
	websocket_gui.onclose = function(event){
		radiConect.contentWindow.postMessage({connection: 'exercise', command: 'down'}, '*');
		if(event.wasClean){
			alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
		}
		else{
			alert("[close] Connection closed!");
		}
	}	

	// What to do when a message from server is received
	websocket_gui.onmessage = function(event){
		operation = event.data.substring(0, 4);
		if(operation == "#gui"){
			// Parse the entire Object
			data = JSON.parse(event.data.substring(4, ));

			// Parse the Map data
			map_data = JSON.parse(data.map);
			paintEvent(map_data.car, map_data.obstacle, map_data.average, map_data.lasers, map_data.ranges);

			// Send the Acknowledgement Message
			websocket_gui.send("#ack");
		}
	
	};
}

var canvas = document.getElementById("gui_canvas");
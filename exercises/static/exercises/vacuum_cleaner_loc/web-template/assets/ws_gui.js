// Set the src of iframe tag
//document.getElementById("gzweb").setAttribute(
//	"src", "http://" + websocket_address + ":8080")

// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui, operation, data;
var pose, content;
var command_input;
var last_data_nav;

function declare_gui(websocket_address){
	websocket_gui = new WebSocket("ws://" + websocket_address + ":2303/");

	websocket_gui.onopen = function(event){
		radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: '6'}, '*');
		if (websocket_code.readyState == 1) {
			alert("[open] Connection established!");
			radiConect.contentWindow.postMessage({connection: 'exercise', command: 'up'}, '*');
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
			// Slice off ( and )
			pose = data.map.substring(1, data.map.length - 1);
			content = pose.split(',').map(function(item) {
				return parseFloat(item);
			})
			draw(content[0], content[1], content[2], content[3]);

			// Parse the Navigation Grid Data
			if (data.nav != null) {
				if (last_data_nav !== data.nav) {
					last_data_nav = data.nav;
					navData = data.nav;
					navData = navData.substring(2, navData.length-2);
					navArray = navData.split(/\[|\]/);
					for (let i = 0; i < navArray.length; i++) {
						if (navArray[i] == ", ") {
							navArray.splice(i, 1);
						}
					}
					for (let i = 0; i < navArray.length; i++) {
						navArray[i] = navArray[i].split(", ");
					}
		
					// Draw the nav data
					reset_evaluator_map();
					initGrid(navArray.length, navArray[0].length);
					fillGrid(navArray);
				}				
			}

			// Send the Acknowledgment Message
			websocket_gui.send("#ack");
		}
		
	}
}


// Image Display Configuration

// var canvas = document.getElementById("gui_canvas"),
//     context = canvas.getContext('2d');
//     image = new Image();
    
// Lap time DOM
// var lap_time_display = document.getElementById("lap_time");

// For image object
// image.onload = function(){
//     update_image();
// }

// Request Animation Frame to remove the flickers
// function update_image(){
// 	window.requestAnimationFrame(update_image);
// 	context.drawImage(image, 0, 0);
// }

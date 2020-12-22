// Set the src of iframe tag
//document.getElementById("gzweb").setAttribute(
//	"src", "http://" + websocket_address + ":8080")

// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui;
function declare_gui(websocket_address){
	websocket_gui = new WebSocket("ws://" + websocket_address + ":2303/");

	websocket_gui.onopen = function(event){
		alert("[open] Connection established!");
	}
	
	websocket_gui.onclose = function(event){
		if(event.wasClean){
			alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
		}
		else{
			alert("[close] Connection closed!");
		}
	}

	// What to do when a message from server is received
	websocket_gui.onmessage = function(event){
		var operation = event.data.substring(0, 4);
		
		if(operation == "#lap"){
			var lap_time = event.data.substring(4, );
			//lap_time_display.textContent = lap_time;
		}
		
		else if(operation == "#map"){
			// To slice off the ()
			var pose = event.data.substring(5, event.data.length - 1);
			var content = pose.split(',').map(function(item) {
				return parseFloat(item);
			})
			draw(content[0], content[1], content[2], content[3]);
		}
		
		else if(operation == "#cop"){
			// Set the value of command
			var command_input = event.data.substring(4, );
			command.value = command_input;
			// Go to next command line
			next_command();
		}
		
		else if(operation == "#cor"){
			// Set the value of command
			var command_input = event.data.substring(4, );
			command.value = command_input;
			// Go to next command line
			next_command();
			// Focus on the next line
			command.focus();
		}
		
		websocket_gui.send("Received!");
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

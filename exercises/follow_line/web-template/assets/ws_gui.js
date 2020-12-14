// Set the src of iframe tag
//document.getElementById("gzweb").setAttribute(
//	"src", "http://" + websocket_address + ":8080")

// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui, animation_id;

function declare_gui(){
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
		
		if(operation == "#gui"){
			// Parse the entire Object
			var data = JSON.parse(event.data.substring(4, ));
			
			// Parse the Image Data
			var image_data = JSON.parse(data.image),
				source = decode_utf8(image_data.image),
				shape = image_data.shape;
				
			canvas.width = shape[1];
			canvas.height = shape[0];
			
			image.src = "data:image/jpeg;base64," + source;
			
			// Parse the Lap data
			var lap_time = data.lap;
			if(lap_time != ""){
				lap_time_display.textContent = lap_time;
			}
			
			// Parse the Map data
			// Slice off ( and )
			var pose = data.map.substring(1, data.map.length - 1);	
			var content = pose.split(',').map(function(item){
				return parseFloat(item);
			})
			drawCircle(content[0], content[1]);
			
			// Send the Acknowledgment Message
			websocket_gui.send("#ack");
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
		
	}
}

var canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');
    image = new Image();
    
// Lap time DOM
var lap_time_display = document.getElementById("lap_time");

// For image object
image.onload = function(){
    update_image();
}

// Request Animation Frame to remove the flickers
function update_image(){
	//window.requestAnimationFrame(update_image);
	context.drawImage(image, 0, 0);
}

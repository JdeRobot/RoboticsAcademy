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
var pose, content;
var command_input;
var count = 0;
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

			// Parse the image data
			image_data = JSON.parse(data.image);

			source = decode_utf8(image_data.image);
			shape = image_data.shape;
			if(source != ""){
				canvas.src = "data:image/jpeg;base64," + source;
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

			// Print Particles
			var particles = JSON.parse(data.particles);
            if(particles != "") {
                printParticles(particles);
            }
            // Print Pose3D
			var pose = JSON.parse(data.pos3D);
            paintPoints3D([[content[0], content[1], 0], pose])
			// Send the Acknowledgment Message
			websocket_gui.send("#ack");
		}

	}
}


// Image Display Configuration
var canvas = document.getElementById("gui_canvas");


// 3D Scene
function paintPoints3D(points){

	var point = [];
	var colors = [[255,0,0], [0,255,0]]
    for (var i = 0; i < points.length; i++) {
    	if (points[i] !== ""){
    		point.x = points[i][0];
    		point.y = points[i][1];
    		point.z = points[i][2];
    		point.r = colors[i][0];
    		point.g = colors[i][1];
    		point.b = colors[i][2];
    		addPoint(point);
		}

    }

}


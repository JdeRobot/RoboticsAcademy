// Set the src of iframe tag
var matching_history = [];
var matching_history_color = [];
var paint_matching = 'false';

// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s));
}

// Websocket and other variables for image display
var websocket_gui, operation, data;
var image_data, source, shape;
var lap_time, pose, content;
var command_input;
function declare_gui(){
    websocket_gui = new WebSocket("ws://" + websocket_address + ":2303/");

    websocket_gui.onopen = function(event){
        alert("[open] Connection established!");
    }

    websocket_gui.onclose = function(event){
        radiConect.contentWindow.postMessage('down', '*');
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
        radiConect.contentWindow.postMessage('up', '*');
        if(operation == "#gui"){
            // Parse the entire Object
			data = JSON.parse(event.data.substring(4, ));
			// Parse the Image Data
			image_data = JSON.parse(data.image);
			source = decode_utf8(image_data.image);
			shape = image_data.shape;

			if(source != ""){
				canvas.src = "data:image/jpeg;base64," + source;
				canvas.width = shape[1];
				canvas.height = shape[0];
			}

            var point = JSON.parse(data.point);
            if(point != "")
            {
                paintPoints(point);
            }
            var matching = JSON.parse(data.matching);

            if(matching != "")
            {
                SetMatching(matching);
            }

            paint_matching = data.paint_matching;
            console.log("ENVIO ACK")
            // Send the Acknowledgment Message
            websocket_gui.send("#ack" + gui_frequency);

        }else if(operation == "#res"){
            // Set the value of command
            reset_scene3d();
            reset_matching();
        }
    }
}


var canvas = document.getElementById("gui_canvas");

function paintPoints(points_received)
{
    var point = [];

    for (var i = 0; i < points_received.length; i++)
    {
        point.x = points_received[i][0];
        point.y = points_received[i][1];
        point.z = points_received[i][2];
        point.r = points_received[i][3];
        point.g = points_received[i][4];
        point.b = points_received[i][5];
		addPoint(point);
    }
}

function getRandomColor() {
  var letters = '0123456789ABCDEF';
  var color = '#';
  for (var i = 0; i < 6; i++) {
    color += letters[Math.floor(Math.random() * 16)];
  }
  return color;
}

function line_matching_image(matching)
{
    context.beginPath();
    context.moveTo(matching.x1, matching.y1);
    context.lineTo(matching.x2 + 320, matching.y2);
    context.arc(matching.x1, matching.y1, 5, 0, 2 * Math.PI);
    context.arc(matching.x2 + 320, matching.y2, 5, 0, 2 * Math.PI);
    context.strokeStyle = matching.color;
    context.fillStyle = matching.color;
    context.fill();
    context.stroke();
}

function paintMatching()
{
    var matching = [];

    for (var i = 0; i < matching_history.length; i++)
    {
        matching.x1 = matching_history[i][0]/2;
        matching.y1 = matching_history[i][1]/2;
        matching.x2 = matching_history[i][2]/2;
        matching.y2 = matching_history[i][3]/2;
        matching.color = matching_history_color[i];

        line_matching_image(matching);
    }
}

function SetMatching(matching_received)
{
    matching_history = matching_history.concat(matching_received);
    for (var i = 0; i < matching_received.length; i++)
    {
        matching_history_color = matching_history_color.concat(getRandomColor());
    }

}

function reset_matching (){
    matching_history = [];
    matching_history_color = [];
    context.drawImage(image1, 0, 0,320, 240);
    context.drawImage(image2, 320, 0,320, 240);
}

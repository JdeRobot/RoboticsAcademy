// Set the src of iframe tag
var matching_history = [];
var matching_history_color = [];
var paint_matching = 'false';

// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s));
}

// Websocket and other variables for image display
var websocket_gui;
var image_data1, image_data2, source1, source2
function declare_gui(){
    websocket_gui = new WebSocket("ws://" + websocket_address + ":2303/");

    websocket_gui.onopen = function(event){
		set_launch_level(get_launch_level()+1);
		if (websocket_code.readyState == 1) {
			alert("[open] Connection established!");
			connectionUpdate({connection: 'exercise', command: 'up'}, '*');
		}
    }

    websocket_gui.onclose = function(event){
        connectionUpdate({connection: 'exercise', command: 'down'}, '*');
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

            image_data1 = JSON.parse(data.img1)
            source1 = decode_utf8(image_data1.img)

            if(source1 !== ""){
                image1.src = "data:image1/jpeg;base64," + source1;
                update_image();
            }
            // Parse the Image Data
            image_data2 = JSON.parse(data.img2)
            source2 = decode_utf8(image_data2.img)

            if(source2 !== ""){
                image2.src = "data:image2/jpeg;base64," + source2;
                update_image();
            }

            var point = JSON.parse(data.pts);
            if(point != "")
            {
                paintPoints(point);
            }
            var matching = JSON.parse(data.match);

            if(matching != "")
            {
                SetMatching(matching);
            }
            // Paint Maching
            paint_matching = data.p_match
            if (paint_matching === "T") {
                paintMatching();
            }
            // Send the Acknowledgment Message
            websocket_gui.send("#ack");

        }else if(operation == "#res"){
            // Set the value of command
            reset_scene3d();
            reset_matching();
        }
    }
}


var canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');
    canvas.height = 240;
    canvas.width = 650;
    image1 = new Image();
    image2 = new Image();


// For image object
image1.onload = function(){
    update_image();
}

image2.onload = function(){
    update_image();
}

// Request Animation Frame to remove the flickers
function update_image(){
    context.drawImage(image1, 0, 0,320, 240);
    context.drawImage(image2, 320, 0,320, 240);
}

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

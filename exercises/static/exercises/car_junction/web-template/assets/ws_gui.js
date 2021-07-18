// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s));
}

// Websocket and other variables for image display
var websocket_gui;
var image_data1, image_data2, source1, source2;

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
        var operation = event.data.substring(0, 4);
        if(operation == "#gui"){
            // Parse the entire Object
            var data = JSON.parse(event.data.substring(4, ));

            // Parse the Image Data
            image_data1 = JSON.parse(data.imageL)
            source1 = decode_utf8(image_data1.img)

            if(source1 !== ""){
                image1.src = "data:image1/jpeg;base64," + source1;
            }

            // Parse the Image Data
            image_data2 = JSON.parse(data.imageC)
            source2 = decode_utf8(image_data2.img)

            if(source2 !== ""){
                image2.src = "data:image2/jpeg;base64," + source2;
            }

            // Parse the Image Data
            image_data3 = JSON.parse(data.imageR)
            source3 = decode_utf8(image_data3.img)

            if(source3 !== ""){
                image3.src = "data:image3/jpeg;base64," + source3;
            }

            var v = JSON.parse(data.v);
            var w = JSON.parse(data.w);

            // Send the Acknowledgment Message
            websocket_gui.send("#ack");
        }
    }
}

var canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');
    canvas.height = 120;
    canvas.width = 490;
    image1 = new Image();
    image2 = new Image();
    image3 = new Image();


// For image object
image1.onload = function(){
    update_image();
}

image2.onload = function(){
    update_image();
}

image3.onload = function(){
    update_image();
}

// Request Animation Frame to remove the flickers
function update_image(){
    window.requestAnimationFrame(update_image);
    context.drawImage(image1, 0, 0,160, 120);
    context.drawImage(image2, 160, 0,160, 120);
    context.drawImage(image3, 320, 0,160, 120);
}

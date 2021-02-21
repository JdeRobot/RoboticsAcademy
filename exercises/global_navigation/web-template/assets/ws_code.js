//Editor Part
var editor = ace.edit("editor");
editor.setTheme("ace/theme/monokai");
editor.session.setMode("ace/mode/python");

var stop_button = document.getElementById("stop");
stop_button.disabled = true;
stop_button.style.opacity = "0.4";
stop_button.style.cursor = "not-allowed";

// running variable for psuedo decoupling 
// Play/Pause from Reset
var frequency = "0",
	running = false;

//WebSocket for Code
var websocket_code;
function declare_code(websocket_address){
	websocket_code = new WebSocket("ws://" + websocket_address + ":1905/");
        
	websocket_code.onopen = function(event){
		alert("[open] Connection established!");
	}
	websocket_code.onclose = function(event){
		if(event.wasClean){
			alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
		}
		else{
			alert("[close] Connection closed!");
		}
	}

	websocket_code.onmessage = function(event){
		var source_code = event.data;
		//operation = source_code.substring(0,);
                //document.getElementById('code').textContent = operation;
                operation = source_code.substring(0,5);
		
		if(operation == "#load"){
			editor.setValue(source_code.substring(5,));
                     
		}
		else if(operation == "#freq"){
			var frequency_message = JSON.parse(source_code.substring(5,));
                        
			// Parse GUI and Brain frequencies
			document.querySelector("#ideal_gui_frequency").value = frequency_message.gui;
			document.querySelector('#ideal_code_frequency').value = frequency_message.brain;
		}
		else if(operation == "#arry"){
                        var data = event.data.substring(5,);
                        //document.getElementById("array").value = frequ;
                        dic = JSON.parse(data);
                        image_data = JSON.parse(dic.image),
                        source = decode_utf8(image_data.image),
                        shape = image_data.shape;
                        if(source != ""){
				image.src = "data:image/png;base64," + source;
				canvas.width = shape[1];
				canvas.height = shape[0];
			}
                        
                        //var arr = data.array;
                        //console.log(arr)
                        localStorage.setItem("array",data);
                }
                        //generatepath(arr);
		// Send the acknowledgment message along with frequency
		code_frequency = document.querySelector('#code_frequency').value;
		gui_frequency = document.querySelector('#gui_frequency').value;
		frequency_message = {"brain": code_frequency, "gui": gui_frequency};
		websocket_code.send("#freq" + JSON.stringify(frequency_message));
	};
}

// Function that sends/submits the code!
function submitCode(){
	// Get the code from editor and add headers
    var python_code = editor.getValue();
    python_code = "#code\n" + python_code
    
    // Get the debug level and add header
	var debug_level = document.querySelector('input[name = "debug"]').value;
    python_code = "#dbug" + debug_level + python_code
    
    console.log("Code Sent! Check terminal for more information!");
    websocket_code.send(python_code);

    stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
	stop_button.style.cursor = "default";
	
	running = true;
}

// Function that send/submits an empty string
function stopCode(){
    var stop_code = "#code\n";
    console.log("Message sent!");
	websocket_code.send(stop_code);
	
	running = false;
}

// Function to save the code
function saveCode(){
	// Get the code from editor and add header
	
	var python_code = editor.getValue();
        
	python_code = "#save" + python_code;
	console.log("Code Sent! Check terminal for more information!");
	websocket_code.send(python_code)
}

// Function to load the code
function loadCode(){
	// Send message to initiate load message
	var message = "#load";
	websocket_code.send(message);
	
}

// Function to command the simulation to reset
function resetSim(){
	// Send message to initiate reset
        
	var message = "#rest"
	websocket_code.send(message)
	if(running == true){
		submitCode();
	}
}

function pickLoc(){
        var data = destinationPicker(event);
        coords = {"data" : data};
        websocket_code.send("#pick" + JSON.stringify(coords));
}
        
 
var canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');
    image = new Image();

function update_image(){
	window.requestAnimationFrame(update_image);
	context.drawImage(image, 0, 0);
}


// Function for range slider
function codefrequencyUpdate(vol) {
	document.querySelector('#code_frequency').value = vol;
}

// Function for range slider
function guifrequencyUpdate(vol) {
	document.querySelector('#gui_frequency').value = vol;
}

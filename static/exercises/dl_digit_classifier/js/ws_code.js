var save_button = document.getElementById("save");
save_button.disabled = true;
save_button.style.opacity = "0.4";
save_button.style.cursor = "not-allowed";

var load_button = document.getElementById("load");
load_button.disabled = true;
load_button.style.opacity = "0.4";
load_button.style.cursor = "not-allowed";

var reset_button = document.getElementById("reset");
reset_button.disabled = true;
reset_button.style.opacity = "0.4";
reset_button.style.cursor = "not-allowed";

// running variable for psuedo decoupling
// Play/Pause from Reset
var frequency = "0",
	running = false;

//WebSocket for Code
var websocket_code;
function declare_code(){
	websocket_code = new WebSocket("ws://" + websocket_address + ":1905/");

	websocket_code.onopen = function(event){
		if (websocket_gui.readyState == 1) {
			connectionUpdate({connection: 'exercise', command: 'up'}, '*');
			alert("[open] Connection established!");
		}
        websocket_code.send("#ping");
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
		operation = source_code.substring(0, 5);

		if(operation == "#load"){
			editor.setValue(source_code.substring(5,));
		}
		else if(operation == "#freq"){
			try {
				var frequency_message = JSON.parse(source_code.substring(5,));
				// Parse GUI and Brain frequencies
				document.querySelector("#ideal_gui_frequency").value = frequency_message.gui;
				document.querySelector('#ideal_code_frequency').value = frequency_message.brain;
				// Send the acknowledgment message along with frequency
				code_frequency = document.querySelector('#code_frequency').value;
				gui_frequency = document.querySelector('#gui_frequency').value;
				frequency_message = {"brain": code_frequency, "gui": gui_frequency};
				websocket_code.send("#freq" + JSON.stringify(frequency_message));
			}			
			catch {}
		}
		else if (operation == "#ping"){
            websocket_code.send("#ping");
        }
	};
}

// Function that sends/submits the code!
function submitCode(){
	document.getElementById("output_heading").textContent = "Uploading model..."
	var input = document.getElementById("dl_model");
	var fReader = new FileReader();
	fReader.readAsDataURL(input.files[0]);
	fReader.onloadend = function(event){
		websocket_code.send(event.target.result);
	}

	running = true;
}

// Function that send/submits an empty string
function stopCode(){
    var stop_inference = "#code\n";
    console.log("Message sent!");
	websocket_code.send(stop_inference);

	running = false;
}

// Function for range slider
function codefrequencyUpdate(vol) {
	document.querySelector('#code_frequency').value = vol;
}
// Function for range slider
function guifrequencyUpdate(vol) {
	document.querySelector('#gui_frequency').value = vol;
}

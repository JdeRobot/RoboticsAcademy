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
function declare_code(){
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
		operation = source_code.substring(0, 5);

		if(operation == "#load"){
			editor.setValue(source_code.substring(5,));
		}
		else if(operation == "#freq"){
			var frequency_message = JSON.parse(source_code.substring(5,));
			// Parse GUI and Brain frequencies
			document.querySelector("#ideal_gui_frequency").value = frequency_message.gui;
			document.querySelector('#ideal_code_frequency').value = frequency_message.brain;
		}

		// Send the acknowledgment message along with frequency
		code_frequency = document.querySelector('#code_frequency').value;
		gui_frequency = document.querySelector('#gui_frequency').value;
		frequency_message = {"brain": code_frequency, "gui": gui_frequency};
		websocket_code.send("#freq" + JSON.stringify(frequency_message));
	};
}


function Upload_Video(){

	var input = document.getElementById("video_file");
	var fReader = new FileReader();
	fReader.readAsDataURL(input.files[0]);
	fReader.onloadend = function(event){
		alert("Uploading video file...");
		websocket_code.send("#save_video" + event.target.result);
		alert("...the video has been sent!");
	}
	
}

function Upload_Model(){
	var input = document.getElementById("dl_model");
	var fReader = new FileReader();
	fReader.readAsDataURL(input.files[0]);
	fReader.onloadend = function(event){
		alert("Uploading DL model file...");
		websocket_code.send("#save_model" + event.target.result);
		alert("...the model has been transferred!");
	}
}


// Function that sends/submits the model for live inference!
function LiveInfer(){
	websocket_code.send("#infer");
	alert("Performing Live Inference. Please wait.... ");
    stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
	stop_button.style.cursor = "default";

	running = true;
}

function VideoInfer(){
	websocket_code.send("#video_infer");
	alert("Performing Video Inference. Please wait....");
    stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
	stop_button.style.cursor = "default";

	running = true;
}

function benchmarkModel(){
	websocket_code.send("#eval");
	alert("Benchmarking Model on Oxford Town Centre Dataset. Please wait....");
	stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
	stop_button.style.cursor = "default";

	running = true;
}

function visualizeModel(){
	websocket_code.send("#visual");
	alert("Opening Visualizer. Please wait....");
}

// Function that send/submits an empty string
function stopCode(){
    var stop_inference = "#stop\n";
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

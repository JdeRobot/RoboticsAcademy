var stop_button = document.getElementById("stop");
var live_button = document.getElementById("Live_Infer");
var benchmark_button = document.getElementById("benchmark");
var video_button = document.getElementById("Video_Infer");
var visualizer_button = document.getElementById("visualizer");
stop_button.disabled = live_button.disabled =  benchmark_button.disabled = video_button.disabled = visualizer_button.disabled = true;
//stop_button.disabled = true;
stop_button.style.opacity = live_button.style.opacity = benchmark_button.style.opacity = video_button.style.opacity = visualizer_button.style.opacity = "0.4";
//stop_button.style.opacity = "0.4";
stop_button.style.cursor = live_button.style.cursor = benchmark_button.style.cursor = video_button.style.cursor = visualizer_button.style.cursor = "not-allowed";
//stop_button.style.cursor = "not-allowed";
var model_uploaded = false;
var video_uploaded = false;
var graph_uploaded = false;


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

		if(operation == "#modl"){
			document.getElementById("output_heading").textContent = "";
			model_uploaded = true;
			live_button.disabled = benchmark_button.disabled = visualizer_button.disabled = false;
			live_button.style.opacity = benchmark_button.style.opacity = visualizer_button.style.opacity = "1.0";
			live_button.style.cursor = benchmark_button.style.cursor = visualizer_button.style.cursor = "default";
			if(video_uploaded == true){
				video_button.disabled = false;
				video_button.style.opacity = "1.0";
				video_button.style.cursor = "default";
			}
		}
		if(operation == "#vido"){
			document.getElementById("output_heading").textContent = "";
			video_uploaded = true;
			if(model_uploaded == true){
				video_button.disabled = false;
				video_button.style.opacity = "1.0";
				video_button.style.cursor = "default";
			}
		}

		if(operation == "#load"){
			editor.setValue(source_code.substring(5,));
		}
		if(operation == "#freq"){
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
		alert("Uploading video file. The buttons will enable after the video and model have been uploaded....");
		websocket_code.send("#save_video" + event.target.result);
		document.getElementById("output_heading").textContent = "Uploading Video....";
		// alert("...the video has been sent!");
	}
	
}

function Upload_Model(){
	var input = document.getElementById("dl_model");
	var fReader = new FileReader();
	fReader.readAsDataURL(input.files[0]);
	fReader.onloadend = function(event){
		alert("Uploading DL model file. The inference button will enable after the file has been uploaded....");
		websocket_code.send("#save_model" + event.target.result);
		document.getElementById("output_heading").textContent = "Uploading DL model....";
		// alert("...the model has been transferred!");
	}
}


// Function that sends/submits the model for live inference!
function LiveInfer(){
	websocket_code.send("#infer");
	alert("Performing Live Inference. Please wait.... ");
	document.getElementById("output_heading").textContent = "Performing Live Inference. Please wait...";
    stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
	stop_button.style.cursor = "default";

	running = true;
}

function VideoInfer(){
	websocket_code.send("#video_infer");
	alert("Performing Video Inference. Please wait....");
	document.getElementById("output_heading").textContent = "Performing Inference on Video. Please wait...";
    stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
	stop_button.style.cursor = "default";

	running = true;
}

function benchmarkModel(){
	websocket_code.send("#eval");
	alert("Benchmarking Model on Oxford Town Centre Dataset. Please wait....");
	document.getElementById("output_heading").textContent = "Benchmarking Model on Oxford Town Centre Dataset. Please wait....";
	stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
	stop_button.style.cursor = "default";

	running = true;
}

function graph_input(){
	var graph= document.getElementById("code-menu-1");
	document.getElementById("output_heading").textContent = "Getting Graphs. Please wait....";
	if (graph =='map')
		websocket_code.send('#graphmap');
	else if (graph=='11pt')
		websocket_code.send('#graph_11');	
	stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
	stop_button.style.cursor = "default";

	running = true;	
}

function EnableGraphInput(){
	
	var id=document.getElementById("output_heading").textContent;
	var graph= document.getElementById("code-menu-1");

	while(id!=="Benchmarking process thread closed!"){
		graph.disabled = true;
	}
	graph.disabled = false;
}

function visualizeModel(){
	websocket_code.send("#visual");
	alert("Opening Visualizer. Please wait....");
	window.open("http://localhost:8081");
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

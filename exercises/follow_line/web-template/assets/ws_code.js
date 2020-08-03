// Load code part

const realFileBtn = document.getElementById("real-file");
const customBtn = document.getElementById("load");
const editorele = ace.edit("editor");
customBtn.addEventListener("click", function(){
	realFileBtn.click();
});
realFileBtn.addEventListener("change", function() {
	if (realFileBtn.value) {
		var fr = new FileReader();
		fr.onload = function(){
			editorele.setValue(fr.result, 1);
		}
		fr.readAsText(this.files[0]);
	
	} 
});
    	
//Editor Part
var editor = ace.edit("editor");
editor.setTheme("ace/theme/monokai");
editor.session.setMode("ace/mode/python");

var stop_button = document.getElementById("stop");
stop_button.disabled = true;
stop_button.style.opacity = "0.4";
stop_button.style.cursor = "not-allowed";

var frequency = "0";

//WebSocket for Code
var websocket_code = new WebSocket("ws://" + websocket_address + ":1905/");

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

// Function that sends/submits the code!
function submitCode(){
	// Get the code from editor and add headers
    var python_code = editor.getValue();
    python_code = "#code\n" + python_code
    
    // Get the debug level and add header
	var debug_level = document.querySelector('input[name = "debug"]').value;
    python_code = "#dbug" + debug_level + python_code
    
    // Add freqeuncy header
    python_code = "#freq" + document.querySelector('#frequency').value + "\n" + python_code;
    
    console.log("Code Sent! Check terminal for more information!");
    websocket_code.send(python_code);

    stop_button.disabled = false;
    stop_button.style.opacity = "1.0";
    stop_button.style.cursor = "default";
}

// Function that send/submits an empty string
function stopCode(){
    var stop_code = "#code\n";
    console.log("Message sent!");
    websocket_code.send(stop_code);
}

// Function to save the code locally
function saveCodeLocally(){
	var python_code = editor.getValue();
	var blob = new Blob([python_code], {type: "text/plain; charset=utf-8"});
	if (window.navigator.msSaveOROpenBlob)
		window.navigator.msSaveOrOpenBlob(blob, "follow_line.py");
	else{
		var a = document.createElement("a"),
		url = URL.createObjectURL(blob);
		a.href = url;
		a.download = "follow_line.py";
		document.body.appendChild(a);
		a.click()
		setTimeout(function(){
			document.body.removeChild(a);
			window.URL.revokeObjectURL(url);
		}, 0);
	}
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
}

// Function for range slider
function frequencyUpdate(vol) {
	document.querySelector('#frequency').value = vol;
}

websocket_code.onmessage = function(event){
	var source_code = event.data;
	operation = source_code.substring(0, 5);
	
	if(operation == "#load"){
		editor.setValue(source_code.substring(5,));
	}
	else if(operation == "#freq"){
		frequency = source_code.substring(5,);
		document.querySelector('#ideal_frequency').value = frequency;
	}
	else if(operation == "#ping"){
		websocket_code.send("#pong")
	}
};


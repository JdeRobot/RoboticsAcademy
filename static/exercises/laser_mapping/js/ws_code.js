//Editor Part
var editor = ace.edit("editor");
editor.setTheme("ace/theme/monokai");
editor.session.setMode("ace/mode/python");

// running variable for psuedo decoupling 
// Play/Pause from Reset
var frequency = "0";

var firstCodeSent = false;

//WebSocket for Code
var websocket_code;
var teleop_switch = false;
ArrayText = new Array();
ArrayText ['something'] = "When Teleoperation is ON, use the following keys: \n 'w' -> Go forward \n 's' -> Stop \n 'a' -> Turn left\n 'd' -> Turn right\n 'q' -> Increase linear speed 10%\n 'z' -> Decrease linear speed 10%\n 'e' -> Increase angular speed 10%\n 'c' -> Decrease angular speed 10%\n";
ArrayText ['nothing'] = "";

function declare_code(websocket_address){
	websocket_code = new WebSocket(websocket_address);

	websocket_code.onopen = function(event){
		connectionUpdate({connection: 'exercise', command: 'launch_level', level: '5'}, '*');
		if (websocket_gui.readyState == 1) {
			alert("[open] Connection established!");
			connectionUpdate({connection: 'exercise', command: 'up'}, '*');
		}
		websocket_code.send("#ping");
	}
	websocket_code.onclose = function(event){
		connectionUpdate({connection: 'exercise', command: 'down'}, '*');
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
			// Send the acknowledgment message along with frequency
			code_frequency = document.querySelector('#code_freq').value;
			gui_frequency = document.querySelector('#gui_freq').value;
			frequency_message = {"brain": code_frequency, "gui": gui_frequency};
			websocket_code.send("#freq" + JSON.stringify(frequency_message));
		}
		else if (operation == "#ping"){
            websocket_code.send("#ping");
        }
		else if (operation == "#exec") {
            toggleSubmitButton(true);
        }
	};
}

// Function that sends/submits the code!
function submitCode(){
	// Get the code from editor and add headers
    var python_code = editor.getValue();
    python_code = "#code\n" + python_code
    
    console.log("Code Sent! Check terminal for more information!");
    websocket_code.send(python_code);

	firstCodeSent = true;
}

// Function that send/submits an empty string
function stopCode(){
    var stop_code = "#code\n";
    console.log("Message sent!");
	websocket_code.send(stop_code);
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

// Function for range slider
function codefrequencyUpdate(vol) {
	document.querySelector('#code_freq').value = vol;
}

// Function for range slider
function guifrequencyUpdate(vol) {
	document.querySelector('#gui_freq').value = vol;
}
function Teleoperation(){
	var message = "#teop"
	teleop_switch = !teleop_switch;
	change_teleop_img();
	if (websocket_code != null)
	websocket_code.send(message)
}
function deactivateTeleop(){
	if (teleop_switch)
		Teleoperation();
}
function keyEvent(event){
	var message = "#key";
	var key = event.key;
	message = message + key;
	if (websocket_code != null)
		websocket_code.send(message);
}
function displayText(text_msg){
	HelpWindow = document.getElementById("switch-label");
	HelpWindow.innerHTML = ArrayText[text_msg];
}
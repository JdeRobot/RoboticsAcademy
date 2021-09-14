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
var frequency = "0";

//WebSocket for Code
var websocket_code_guest;
function declare_code_guest(websocket_address){
    websocket_code_guest = new WebSocket("ws://" + websocket_address + ":1904/");

    websocket_code_guest.onopen = function(event){
		radiConect.contentWindow.postMessage({connection: 'exercise', command: 'launch_level', level: '5'}, '*');
		if (websocket_gui_guest.readyState == 1) {
			alert("[open] Connection established!");
			radiConect.contentWindow.postMessage({connection: 'exercise', command: 'up'}, '*');
		}
    }
    websocket_code_guest.onclose = function(event){
        if(event.wasClean){
            alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
        }
        else{
            alert("[close] Connection closed!");
        }
    }

    websocket_code_guest.onmessage = function(event){
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
            // Parse real time factor
			document.querySelector('#real_time_factor').value = frequency_message.rtf;
        }

        // Send the acknowledgment message along with frequency
        code_frequency = document.querySelector('#code_freq').value;
		gui_frequency = document.querySelector('#gui_freq').value;
        real_time_factor = document.querySelector('#real_time_factor').value;

		frequency_message = {"brain": code_frequency, "gui": gui_frequency};
		websocket_code_guest.send("#freq" + JSON.stringify(frequency_message));
    };
}

// Get the code from the server
function getCode(user){
	let path;
	var req = new XMLHttpRequest();
    var server = window.location.host;

	// cambio exercise por exercise template que es donde se almacena en aws
	path = window.location.protocol+'//'+"127.0.0.1:8000"+'/exercise/request/'+exercise+'?diff='+user;

	console.log('PATH: ' ,path);

	req.open("GET", path, false );
	req.send( null );

	return req.responseText;
}

// Function that sends/submits the code!
function submitCodeMouse(){
    try {
        let selector = document.getElementById("mouse");
        let difficulty = selector.options[selector.selectedIndex].value;
        console.log('Dificultad: ', difficulty);
        // Get the code from editor and add headers
        var python_code = getCode(difficulty);
        python_code = "#code\n" + python_code;
        console.log(python_code);

        // Get the debug level and add header
		//var debug_level = document.querySelector('input[name = "debug"]').value;
		var debug_level = 2;
		python_code = "#dbug" + debug_level + python_code

        websocket_code_guest.send(python_code);
        console.log("Code Sent! Check terminal for more information!");

        stop_button.disabled = false;
        stop_button.style.opacity = "1.0";
        stop_button.style.cursor = "default";
    }
	catch (e){
		alert("Connection must be established before sending the code: ", e)
	}
}
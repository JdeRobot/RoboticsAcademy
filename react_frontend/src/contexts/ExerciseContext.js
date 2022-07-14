import { createContext, useState } from "react";

const ExerciseContext = createContext();

export function ExerciseProvider({ children }){
    var ws_manager;
    var address_code;
    var address_gui;
    const [gazeboToggle, setGazeboToggle] = useState(false);
    const [gazeboOn, setGazeboOn] = useState(false);
    const [simReset, setSimReset] = useState(false);
    const [simStop, setSimStop] = useState(false);
    const [simResume, setSimResume] = useState(false);
    const [sendCode, setSendCode] = useState(false);
    const [firstAttempt, setFirstAttempt] = useState(true);
    const [swapping, setSwapping] = useState(false);


	function startSim(step,connectionButton){
    	var level = 0;
    	let websockets_connected = false;

    	if (step == 0) {
    		connectionButton.removeClass("btn-danger").addClass("btn-warning");
			connectionButton.html('<span id="loading-connection" class="fa fa-refresh fa-spin"></span> Connecting');
        	address_code = "ws://" + websocket_address + ":1905";
        	address_gui = "ws://" + websocket_address + ":2303";
        	ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    	} else if (step == 1) {
        	connectionUpdate({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        	var size = get_novnc_size();
        	ws_manager.send(JSON.stringify({
            "command": "open", "exercise": exercise, "width": size.width.toString(), "height": size.height.toString(), "circuit": circuit}));
        	level++;
        	connectionUpdate({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        	ws_manager.send(JSON.stringify({"command" : "Pong"}));
        	console.log("start exercise");
    	} else if (step == 2) {
        	ws_manager.send(JSON.stringify({"command": "exit", "exercise": ""}));
        	stopSimulation();
    	}

    	ws_manager.onopen = function (event) {
        	level++;
        	connectionUpdate({connection: 'manager', command: 'up'}, '*');
        	connectionUpdate({connection: 'exercise', command: 'available'}, '*');
    	}

    	ws_manager.onclose = function (event) {
        	connectionUpdate({connection: 'manager', command: 'down'}, '*');
        	if (!first_attempt) {
            	alert("Connection lost, retrying connection...");
            	startSim(step, circuit, websocket_address,server, username);
        	} else {
        		setFirstAttempt(false);

        	}
    	}

    	ws_manager.onerror = function (event) {
        	connectionUpdate({connection: 'manager', command: 'down'}, '*');
    	}

    	ws_manager.onmessage = function (event) {
        //console.log(event.data);
        	if (event.data.level > level) {
            	level = event.data.level;
            	connectionUpdate({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
        	}
        	if (event.data.includes("Ping")) {
            	if (!websockets_connected && event.data == "Ping3") {
                	level = 4;
                	connectionUpdate({connection: 'exercise', command: 'launch_level', level: `${level}`}, '*');
                	websockets_connected = true;
                	declare_code(address_code);
                	declare_gui(address_gui);
            	}
            	if (gazeboToggle) {
                	console.log("toggle gazebo");
                	if (gazeboOn) {
                    	ws_manager.send(JSON.stringify({"command" : "startgz"}));
                	} else {
                    ws_manager.send(JSON.stringify({"command" : "stopgz"}));
                	}

                	gazeboToggle = false;
            	} else if (simStop){
                	ws_manager.send(JSON.stringify({"command": "stop"}));
                	simStop = false;
                	running = false;
            	}else if (simReset){
                	console.log("reset simulation");
                	ws_manager.send(JSON.stringify({"command": "reset"}));
                	simReset = false;
            	}else if (sendCode) {
                	let python_code = editor.getValue();
		        	python_code = "#code\n" + python_code;
                	ws_manager.send(JSON.stringify({"command": "evaluate", "code": python_code}));
                	sendCode = false;
            	} else if (simResume){
                	ws_manager.send(JSON.stringify({"command": "resume"}));
                	simResume = false;
                	running = true;
            	} else {
                	setTimeout(function () {
                    	ws_manager.send(JSON.stringify({"command" : "Pong"}));
                	}, 1000)
            	}
        	}
        	if (event.data.includes("evaluate")) {
            	if (event.data.length < 9) {    // If there is an error it is sent along with "evaluate"
                	submitCode();
            	} else {
                	let error = event.data.substring(10,event.data.length);
                	connectionUpdate({connection: 'exercise', command: 'error', text: error}, '*');
            	}
            	setTimeout(function () {
                	ws_manager.send(JSON.stringify({"command" : "Pong"}));
            	}, 1000)
        	} else if (event.data.includes("reset")) {
            	ResetEvaluator();
        	} else if (event.data.includes("PingDone")) {
            	enableSimControls();
            	if (resetRequested == true) {
                	resetRequested = false;
            	}
        	}
        	else if (event.data.includes("style")) {
            	let error = event.data.substring(5, event.data.length);
            	connectionUpdate({connection: 'exercise', command: 'style', text: error}, '*');
        	}
    	}
	}

	function toggleGazebo() {
    	if (gazeboOn) {
    		setGazeboOn(false);
    	} else {
           setGazeboOn(true);
    	}
    	setGazeboToggle(true);
	}

	function resetSimulation() {
		setSimReset(true);
	}

	function stopSimulation() {
		setSimStop(true);
	}

	function resumeSimulation() {
		setSimResume(true);
	}

	function checkCode() {
		setSendCode(true);
	}

	function connectionUpdate(data){
			if (data.connection == 'manager') {
				if (data.command == 'up') {
					$("#connection-button").removeClass("btn-warning btn-secondary").addClass("btn-success");
					$("#connection-button").html('<span id="loading-connection" class="bi bi-arrow-down-up"></span> Connected');
					$("#connection-button").prop('disabled', true);
					$("#launch-button").prop('disabled', false);
				}else if (data.command == 'down'){
					$("#connection-button").removeClass("btn-success btn-warning").addClass("btn-secondary");
					$("#connection-button").html('<span id="loading-connection" class="bi bi-arrow-down-up"></span> Connect');
					$("#connection-button").prop('disabled', false);
					if (websocket_code != null)
						websocket_code.close();
					if (websocket_gui != null)
						websocket_gui.close();
					$("#launch-button").removeClass("btn-success btn-warning").addClass("btn-secondary");
					$("#launch-button").html('<span id="loading-connection" class="bi bi-arrow-down-up"></span> Launch');
				}
			} else if (data.connection == 'exercise') {
				if (data.command == 'available') {
					$("#launch-button").removeClass('btn-secondary').addClass('btn-secondary');
				}else if (data.command == 'up') {
					stop();
					swapping = false;
					$("#launch-button").removeClass("btn-warning").addClass("btn-success");
					$("#launch-button").html('<span id="loading-connection" class="bi bi-arrow-down-up"></span> Ready');
					$("#launch-button").prop('disabled', true);
					togglePlayPause(false);
					let reset_button = document.getElementById("reset");
					reset_button.disabled = false;
					reset_button.style.opacity = "1.0";
					reset_button.style.cursor = "default";
					let load_button = document.getElementById("loadIntoRobot");
					load_button.disabled = false;
					load_button.style.opacity = "1.0";
					load_button.style.cursor = "default";
				}else if (data.command == 'down'){
					if (!swapping) {
						$("#launch-button").removeClass("btn-success").addClass("btn-secondary");
						$("#launch-button").html('<span id="loading-connection" class="bi bi-arrow-down-up"></span> Launch');
						$("#launch-button").prop('disabled', false);
					}
				}else if (data.command == 'swap'){
					$("#launch-button").removeClass("btn-success btn-warning btn-secondary").addClass("btn-warning");
					$("#launch-button").html(`<span id="loading-connection" class="fa fa-refresh fa-spin"></span> Launching`);
				}else if (data.command == 'launch_level'){
					let level = data.level;
					$("#launch-button").html(`<span id="loading-connection" class="fa fa-refresh fa-spin"></span> Launching <a id="launch_level">${level}</a>`);
				}else if (data.command == 'error') {
					$('#errorModal .modal-header .modal-header-text').text("Errors detected:");
                    $('#errorModal .modal-body').text(data.text);
                    $('#errorModal').modal({ show:true, backdrop: false});
					$('#errorModal .modal-dialog').draggable({});
					toggleSubmitButton(true);
                }
				else if (data.command == 'style') {
					$('#errorModal .modal-header .modal-header-text').text("Style evaluation:");
					if (data.text.replace(/\s/g, '').length)
                    	$('#errorModal .modal-body').text(data.text);
					else
						$('#errorModal .modal-body').text("Everything is correct!");
                    $('#errorModal').modal({ show:true, backdrop: false});
					$('#errorModal .modal-dialog').draggable({});
                }
			}
		}

    return(
    	<ExerciseContext.Provider value={{ }}>{children}</ExerciseContext.Provider>
	);
}

export default ExerciseContext;
// Function to start car
var playcar_old_timestamp = 0;
function playcar(){
	if(playcar_old_timestamp == 0 || playcar_old_timestamp + 2000 < (new Date).getTime()){
	    var message = "#car" + document.getElementById('car').value;
	    console.log("Message sent: " + message);
	    websocket_gui.send(message);
	    playcar_old_timestamp = (new Date).getTime();
	}
}
// Function to stop car
var stopcar_old_timestamp = 0;
function stopcar(){
	if(stopcar_old_timestamp == 0 || stopcar_old_timestamp + 2000 < (new Date).getTime()){
	    var message = "#stp";
	    console.log("Message sent: " + message);
	    websocket_gui.send(message);
	    stopcar_old_timestamp = (new Date).getTime();
	}
}
// Function to reset car
var resetcar_old_timestamp = 0;
function resetcar(){
	if(resetcar_old_timestamp == 0 || resetcar_old_timestamp + 2000 < (new Date).getTime()){
	    var message = "#rst";
	    console.log("Message sent: " + message);
	    websocket_gui.send(message);
	    resetcar_old_timestamp = (new Date).getTime();
	}
}
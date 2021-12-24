// Function to start turtlebot
var playturtlebot_old_timestamp = 0;
function playturtlebot(){
	if(playturtlebot_old_timestamp == 0 || playturtlebot_old_timestamp + 2000 < (new Date).getTime()){
	    // Send message to initiate start turtlebot
	    var message = "#tur";
	    console.log("Message sent: " + message);
	    websocket_gui.send(message);
	    playturtlebot_old_timestamp = (new Date).getTime();
	}
}
// Function to takeoff turtlebot
var stopturtlebot_old_timestamp = 0;
function stopturtlebot(){
	if(stopturtlebot_old_timestamp == 0 || stopturtlebot_old_timestamp + 2000 < (new Date).getTime()){
	    // Send message to initiate start turtlebot
	    var message = "#stp";
	    console.log("Message sent: " + message);
	    websocket_gui.send(message);
	    stopturtlebot_old_timestamp = (new Date).getTime();
	}
}
// Function to land turtlebot
var resetturtlebot_old_timestamp = 0;
function resetturtlebot(){
	if(resetturtlebot_old_timestamp == 0 || resetturtlebot_old_timestamp + 2000 < (new Date).getTime()){
	    // Send message to initiate start turtlebot
	    var message = "#rst";
	    console.log("Message sent: " + message);
	    websocket_gui.send(message);
	    resetturtlebot_old_timestamp = (new Date).getTime();
	}
}
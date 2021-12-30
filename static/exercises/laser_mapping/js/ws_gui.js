
var enable_flag = 0;
// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui;
function declare_gui(websocket_address){
	websocket_gui = new WebSocket(websocket_address);

	websocket_gui.onopen = function(event){
		set_launch_level(get_launch_level()+1);
		if (websocket_code.readyState == 1) {
			alert("[open] Connection established!");
			connectionUpdate({connection: 'exercise', command: 'up'}, '*');
		}
	}
	
	websocket_gui.onclose = function(event){
		connectionUpdate({connection: 'exercise', command: 'down'}, '*');
		if(event.wasClean){
			alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
		}
		else{
			alert("[close] Connection closed!");
		}
	}

	// What to do when a message from server is received
	websocket_gui.onmessage = function(event){
		/*Extrae caracteres desde un indiceA hasta un indiceB sin incluirlo */
		var operation = event.data.substring(0, 4); /*Devuelve un subconunto de un objeto String*/
		
		if(operation == "#gui"){
			//console.log(event.data.substring(4, ));
			// Parse the entire Object
			/*Analiza una cadena de texto como JSON, transformando opcionalmente el valor producido por el analisis */
			var data;
			// Nan and infinity json errors
			try {
				data = JSON.parse(event.data.substring(4, ));
			} catch (e)
			{
				let json_data = event.data.substring(4, );
				const regex1 = /NaN/g;
				const regex2 = /Infinity/g;
				const regex3 = /-0/g;
				json_data = json_data.replace(regex1, '0');
				json_data = json_data.replace(regex2, '0');
				json_data = json_data.replace(regex3, '0');
				data = JSON.parse(json_data);
			}
			// var pose = data.map.substring(1, data.map.length - 1);
			// var content = pose.split(',').map(function(item) {
			// 	return parseFloat(item);
			// })
			var robot_coord = data.robot_coord;
			var robot_cont = data.robot_contorno;
			var laser_data = data.laser;
			var laser_global = data.laser_global;
			var sonar_sensor_point = data.sonar_sensor;
			var pos_vertices = data.pos_vertices;
			/*Draw all*/
			draw(robot_coord, robot_cont, laser_data, sonar_sensor_point, pos_vertices, laser_global);
			/*If enabled draw the mapping*/
			if(enable_flag == 1){
				drawMapping(laser_data, robot_coord);
			}
			else{
				enable_flag = 0;
			}

			// Send the Acknowledgment Message
			websocket_gui.send("#ack");
		}
		
		else if(operation == "#cor"){
			// Set the value of command
			var command_input = event.data.substring(4, );
			command.value = command_input;
			// Go to next command line
			next_command();
			// Focus on the next line
			command.focus();
		}
		
	}
}

function changeMapping(){
	var mapping_display = document.getElementById("mapping").style.display
	console.log(mapping_display)
	if(mapping_display == "none" || mapping_display == "none"){
	  document.getElementById('mapping').style.display = 'block';
	  enable_flag = 1;
	} else{
	  document.getElementById('mapping').style.display = 'none';
	  enable_flag = 0;
	}
  }
'use strict';
var map;
// Get the select element
const map_selector = document.querySelector('#map_selector');
map_selector.addEventListener('change', function(){
    // Get the value selected
    map = map_selector.options[map_selector.selectedIndex].value;
    console.log('Changed: ', map);
    // Classes
    let classes = ['default', 'prius_map'];

    // Disable connection button
    //$("#connection-button").prop('disabled', true);


    // Set variable to toggle gazebo
    gazeboToggle = true;
    // Stop the simulation
    stop();
    stopBrain();
    alert('Loading map. Please wait until the connection is restored.');
    connectionUpdate({connection: 'exercise', command: 'down'}, '*');
});
function startNewMap() {
    // Kill actual sim
    startSim(2)
    // StartSim
    swapping = true;
    startSim(1, map, "{{websocket_address}}","{{server}}", "{{user.username}}");
    connectionUpdate({connection: 'exercise', command: 'swap'}, '*');
    toggleSubmitButton(false);
    firstCodeSent = false;
}

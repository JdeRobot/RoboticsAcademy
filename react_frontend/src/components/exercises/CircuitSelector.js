import * as React from 'react';
import {FormControl, InputLabel, MenuItem, Select} from "@mui/material";

export default function CircuitSelector(props) {
    const circuitSelector = React.useRef(null);
    function handleChange(e) {

        // let circuit = circuitSelector.current.options[circuit_selector.selectedIndex].value;
        let circuit = e.target.value;
        circuitSelector.current.value = circuit;
        console.log('Changed: ', circuit);
    // Classes
        let classes = ['default', 'montreal', 'montmelo', 'nbg'];

    // Disable connection button
    //$("#connection-button").prop('disabled', true);
    // Set birds-eye background
        let canvas = document.querySelector('#birds-eye');
        classes.forEach(c => {
            canvas.classList.remove(c);
        });
        canvas.classList.add(circuit);

    // Set variable to toggle gazebo
        gazeboToggle = true;
    // Stop the simulation
        stop();
    // Kill actual sim
        startSim(2)
    // StartSim
        startSim(1, circuit);
        alert('Loading circuit. Please wait until the connection is restored.');
    }
    return (
        <FormControl sx = {{ marginX: 20, marginBottom:5 }} color={"secondary"}>
            <InputLabel id="circuit_selector">Circuit</InputLabel>
            <Select
                defaultValue={"default"}
                labelId="circuit_selector"
                id="circuit_selector"
                ref={circuitSelector}
                label="Circuit"
                onChange={handleChange}
            >
                <MenuItem value={"default"} >Default Line</MenuItem>
                <MenuItem value={"montmelo"}>Montmelo Line</MenuItem>
                <MenuItem value={"montreal"}>Montreal Line</MenuItem>
                <MenuItem value={"nbg"}>Nürburgring Line</MenuItem>
            </Select>
        </FormControl>
    );
}

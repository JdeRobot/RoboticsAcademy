import { createContext, useState } from "react";

const CircuitSelectorContext = createContext();

export function CircuitSelectorProvider({ children }){
    const [ circuit, setCircuit] = useState();


    return(
    	<CircuitSelectorContext.Provider value={{ }}>{children}</CircuitSelectorContext.Provider>
	);
}

export default CircuitSelectorContext;
import React, { createContext, useContext } from "react";
import { CommsManager } from "jderobot-commsmanager";

interface ExerciseProviderProps {
  manager: CommsManager | null;
  code?: string;
  children?: React.ReactNode;
}

const defaultCode = `import WebGUI
import HAL
import Frequency
# Enter sequential code!

while True:
    # Enter iterative code!
    Frequency.tick()
`;

export interface ExerciseContextInt {
  manager: CommsManager | null;
  code: string;
}

const defaultContext: ExerciseContextInt = {
  manager: null,
  code: defaultCode,
};

const ExerciseContext = createContext(defaultContext);
export const useExercise = () => useContext(ExerciseContext) ?? defaultContext;

export const ExerciseProvider = ({
  manager,
  code,
  children,
}: ExerciseProviderProps) => {
  return (
    <ExerciseContext.Provider
      value={{ manager: manager, code: code ? code : "" }}
    >
      {children}
    </ExerciseContext.Provider>
  );
};

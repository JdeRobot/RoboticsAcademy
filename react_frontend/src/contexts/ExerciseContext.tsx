import React, { createContext, useContext } from "react";
import { CommsManager } from "jderobot-commsmanager";

interface ExerciseProviderProps {
  manager: CommsManager | null;
  children?: React.ReactNode;
}

export interface ExerciseContextInt {
  manager: CommsManager | null;
}

const defaultContext: ExerciseContextInt = {
  manager: null,
};

const ExerciseContext = createContext(defaultContext);
export const useExercise = () => useContext(ExerciseContext) ?? defaultContext;

export const ExerciseProvider = ({
  manager,
  children,
}: ExerciseProviderProps) => {
  return (
    <ExerciseContext.Provider value={{ manager: manager }}>
      {children}
    </ExerciseContext.Provider>
  );
};

export interface Exercise {
  exercise_id: string;
  name: string;
  description: string;
  tags: string;
  status: ExerciseStatus;
}

export interface ExerciseData {
  name: string;
  exercise_id: string;
  url: string;
  tools: string[];
  tags: string[];
  worlds: object[];
}

export type ExerciseStatus = "ACTIVE" | "INACTIVE" | "PROTOTYPE";
export type Filters = "name" | "tags" | "description" | "status";

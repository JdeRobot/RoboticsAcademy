export interface Exercise {
  exercise_id: string;
  name: string;
  description: string;
  tags: string;
  status: "ACTIVE" | "INACTIVE" | "PROTOTYPE";
}

export type Filters = "name" | "tags" | "description" | "status"

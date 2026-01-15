import axios, { AxiosResponse } from "axios";
import { Exercise, ExerciseData } from "Types/exercises";

const isSuccessful = (response: AxiosResponse) => {
  return response.status >= 200 && response.status < 300;
};

const getCookie = (name: string) => {
  const value = `; ${document.cookie}`;
  const parts = value.split(`; ${name}=`); //to get CSRF token among all the cookies in 'value'
  if (parts.length === 2) return parts.pop()?.split(";").shift();
  return undefined;
};

const csrfToken = getCookie("csrftoken");
const axiosExtra = {
  headers: {
    "X-CSRFToken": csrfToken,
  },
};

const getProjectData = async (projectId?: string): Promise<any> => {
  if (!projectId) throw new Error("Current Project ID is not set");

  const apiUrl = `/academy/get_info/?project_id=${projectId}`;
  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to create app."); // Response error
  }

  return response.data.info;
};

const getExerciseList = async (): Promise<Exercise[]> => {

  const apiUrl = `/academy/get_exercise_list/`;
  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to create app."); // Response error
  }

  return response.data.exercises;
};

export {getProjectData, getExerciseList};
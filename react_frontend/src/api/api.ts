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

const getProjectData = async (
  projectId?: string
): Promise<Omit<ExerciseData, "universes">> => {
  if (!projectId) throw new Error("Current Project ID is not set");

  const apiUrl = `/academy/get_info/?project_id=${projectId}`;
  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to create app."); // Response error
  }

  const data = response.data.info;
  data["exercise_id"] = projectId;

  return data;
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

const listUniverses = async (project: string) => {
  if (!project) throw new Error("Current Project id is not set");

  const apiUrl = `/academy/get_universes_list?project=${encodeURIComponent(
    project
  )}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get universes.");
  }

  return response.data.universes_list;
};

const getRoboticsBackendUniverse = async (
  project: string,
  universe: string
) => {
  if (!project) throw new Error("Current Project id is not set");

  const apiUrl = `/academy/get_docker_universe_data?universe=${encodeURIComponent(
    universe
  )}&project=${encodeURIComponent(project)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(
      response.data.message || "Failed to retrieve universe config"
    ); // Response error
  }

  return {
    world: response.data.universe.world,
    robot: response.data.universe.robot,
    tools: response.data.universe.tools,
    tools_config: response.data.universe.tools_config,
  };
};

const getHelperFile = async (
  project: string,
  language: string,
  fileName: string
) => {
  if (!project) throw new Error("Current Project id is not set");
  if (!language) throw new Error("Current Language is not set");
  if (!fileName) throw new Error("File name is not set");

  const apiUrl = `/academy/get_helper_file?project=${encodeURIComponent(
    project
  )}&language=${encodeURIComponent(language)}&filename=${encodeURIComponent(
    fileName
  )}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get file list."); // Response error
  }

  return response.data.content;
};

const getHelperFileList = async (project: string, language: string) => {
  if (!project) throw new Error("Current Project id is not set");
  if (!language) throw new Error("Current Language is not set");

  const apiUrl = `/academy/get_helper_file_list?project=${encodeURIComponent(
    project
  )}&language=${encodeURIComponent(language)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get file list."); // Response error
  }

  return response.data.file_list;
};

const getFileList = async (project: string) => {
  if (!project) throw new Error("Current Project id is not set");

  // TODO:add whitelist parameter

  const apiUrl = `/academy/get_file_list?project=${encodeURIComponent(
    project
  )}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get file list."); // Response error
  }

  return response.data.file_list;
};

const getFile = async (
  project: string,
  fileName: string,
) => {
  if (!project) throw new Error("Project name is not set");
  if (!fileName) throw new Error("File name is not set");

  const apiUrl = `/academy/get_file?project=${encodeURIComponent(project)}&filename=${encodeURIComponent(fileName)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get file list."); // Response error
  }

  return response.data.content;
};

const saveFile = async (
  project: string,
  fileName: string,
  content: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("Current File name is not set");

  const apiUrl = "/academy/save_file/";

  const params = {
    project: project,
    content: content,
    filename: fileName,
  };

  try {
    const response = await axios.post(apiUrl, params, axiosExtra);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error) {
    console.log(error);
    throw error; // Rethrow
  }
};


export {
  getProjectData,
  getHelperFileList,
  getHelperFile,
  getExerciseList,
  getRoboticsBackendUniverse,
  listUniverses,
  getFile,
  saveFile,
  getFileList
};

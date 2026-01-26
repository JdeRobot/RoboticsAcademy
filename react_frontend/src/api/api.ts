import axios, { AxiosError, AxiosResponse } from "axios";
import { Entry } from "jderobot-ide-interface";
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

const getHelperFileList = async (
  project: string,
  language: string
): Promise<Entry[]> => {
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

  return JSON.parse(response.data.file_list);
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

const getFile = async (project: string, fileName: string, binary?: boolean) => {
  if (!project) throw new Error("Project name is not set");
  if (!fileName) throw new Error("File name is not set");

  let apiUrl = `/academy/get_file?project=${encodeURIComponent(
    project
  )}&filename=${encodeURIComponent(fileName)}`;

  if (binary) apiUrl += `&binary=true`;

  try {
    const response = await axios.get(apiUrl);
    if (binary) {
      return atob(response.data.content);
    }
    return response.data.content;
  } catch (e: unknown) {
    const error = e as AxiosError<any, Record<string, unknown>>;
    throw Error(error.response?.data.message);
  }
};

const saveFile = async (project: string, fileName: string, content: string) => {
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

const getTeaser = async (project: string) => {
  if (!project) throw new Error("Project name is not set");

  const apiUrl = `/academy/get_exercise_teaser?project=${encodeURIComponent(
    project
  )}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get file list."); // Response error
  }

  return `data:image/png;base64,${response.data}`;
};

const createFile = async (
  projectId: string,
  fileName: string,
  location: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location === undefined) throw new Error("Location is not set");

  const apiUrl = "/academy/create_file/";

  const params = {
    project_id: projectId,
    location: location,
    file_name: fileName,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra);
    return;
  } catch (e: unknown) {
    const error = e as AxiosError<any, Record<string, unknown>>;
    throw Error(error.response?.data.message);
  }
};

const renameFile = async (
  projectId: string,
  path: string,
  new_path: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");
  if (!new_path) throw new Error("New path is not set");

  const apiUrl = "/academy/rename_file/";

  const params = {
    project_id: projectId,
    path: path,
    rename_to: new_path,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra);
    return;
  } catch (e: unknown) {
    const error = e as AxiosError<any, Record<string, unknown>>;
    throw Error(error.response?.data.message);
  }
};

const deleteFile = async (projectId: string, path: string) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");

  const apiUrl = "/academy/delete_file/";

  const params = {
    project_id: projectId,
    path: path,
  };

  const response = await axios.post(apiUrl, params, axiosExtra);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to upload file."); // Response error
  }
};

const uploadFile = async (
  projectId: string,
  fileName: string,
  location: string,
  content: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location === undefined) throw new Error("Location is not set");
  if (!content) throw new Error("Content is not defined");

  const apiUrl = "/academy/upload/";
  const params = {
    project_id: projectId,
    file_name: fileName,
    location: location,
    content: content,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra);
    return;
  } catch (e: unknown) {
    const error = e as AxiosError<any, Record<string, unknown>>;
    throw Error(error.response?.data.message);
  }
};

const createFolder = async (
  projectId: string,
  location: string,
  folderName: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!folderName) throw new Error("Folder name is not set");
  if (location === undefined) throw new Error("Location is not set");

  const apiUrl = "/academy/create_folder/";

  const params = {
    project_id: projectId,
    location: location,
    folder_name: folderName,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra);
    return;
  } catch (e: unknown) {
    const error = e as AxiosError<any, Record<string, unknown>>;
    throw Error(error.response?.data.message);
  }
};

const renameFolder = async (
  projectId: string,
  path: string,
  new_path: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");
  if (!new_path) throw new Error("New path is not set");

  const apiUrl = "/academy/rename_folder/";

  const params = {
    project_id: projectId,
    path: path,
    rename_to: new_path,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra);
    return;
  } catch (e: unknown) {
    const error = e as AxiosError<any, Record<string, unknown>>;
    throw Error(error.response?.data.message);
  }
};

const deleteFolder = async (projectId: string, path: string) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");

  const apiUrl = "/academy/delete_folder/";

  const params = {
    project_id: projectId,
    path: path,
  };

  const response = await axios.post(apiUrl, params, axiosExtra);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to upload file."); // Response error
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
  getFileList,
  getTeaser,
  createFile,
  renameFile,
  deleteFile,
  uploadFile,
  createFolder,
  renameFolder,
  deleteFolder,
};

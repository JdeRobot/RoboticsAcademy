import axios, { AxiosError } from "axios";
import { Entry } from "jderobot-ide-interface";
import { Exercise, ExerciseData } from "Types/exercises";

const getCookie = (name: string) => {
  const value = `; ${document.cookie}`;
  const parts = value.split(`; ${name}=`); //to get CSRF token among all the cookies in 'value'
  if (parts.length === 2) return parts.pop()?.split(";").shift();
  return undefined;
};

const axiosExtra = () => ({
  headers: {
    "X-CSRFToken": getCookie("csrftoken"),
  },
});

type ApiError = AxiosError<Record<string, string>, Record<string, unknown>>;

const getProjectData = async (
  projectId?: string
): Promise<Omit<ExerciseData, "universes">> => {
  if (!projectId) throw new Error("Current Project ID is not set");

  const apiUrl = `/academy/enter_exercise/?project_id=${projectId}`;

  try {
    const response = await axios.get(apiUrl);
    const data = response.data.info;
    data["exercise_id"] = projectId;

    return data;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const exitProject = async () => {
  const apiUrl = `/academy/exit_exercise/`;

  try {
    const data = new FormData();
    const csfr = getCookie("csrftoken");
    if (csfr !== undefined) {
      data.append("csrfmiddlewaretoken", csfr);
      navigator.sendBeacon(apiUrl, data);
    }
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getExerciseList = async (): Promise<Exercise[]> => {
  const apiUrl = `/academy/get_exercise_list/`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.exercises;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const listUniverses = async (project: string) => {
  if (!project) throw new Error("Current Project id is not set");

  const apiUrl = `/academy/get_universes_list?project=${encodeURIComponent(
    project
  )}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.universes_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getRoboticsBackendUniverse = async (
  project: string,
  universe: string
) => {
  if (!project) throw new Error("Current Project id is not set");

  const apiUrl = `/academy/get_docker_universe_data?universe=${encodeURIComponent(
    universe
  )}&project=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);
    return {
      world: response.data.universe.world,
      robot: response.data.universe.robot,
      tools: response.data.universe.tools,
      tools_config: response.data.universe.tools_config,
    };
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getHelperFile = async (
  project: string,
  language: string,
  fileName: string,
  binary?: boolean
) => {
  if (!project) throw new Error("Current Project id is not set");
  if (!language) throw new Error("Current Language is not set");
  if (!fileName) throw new Error("File name is not set");

  let apiUrl = `/academy/get_helper_file?project=${encodeURIComponent(
    project
  )}&language=${encodeURIComponent(language)}&filename=${encodeURIComponent(
    fileName
  )}`;

  if (binary) apiUrl += `&binary=true`;

  try {
    const response = await axios.get(apiUrl);
    if (binary) {
      return atob(response.data.content);
    }
    return response.data.content;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
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

  try {
    const response = await axios.get(apiUrl);
    return JSON.parse(response.data.file_list);
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getFileList = async (project: string) => {
  if (!project) throw new Error("Current Project id is not set");

  // TODO:add whitelist parameter

  const apiUrl = `/academy/get_file_list?project=${encodeURIComponent(
    project
  )}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.file_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
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
    const error = e as ApiError;
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
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const createFile = async (
  projectId: string,
  fileName: string,
  location: string,
  template?: string,
  warning?: (msg: string) => void
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location === undefined) throw new Error("Location is not set");

  const apiUrl = "/academy/create_file/";

  const params = {
    project_id: projectId,
    location: location,
    file_name: fileName,
    template: template,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;

    if (warning !== undefined && error.response?.status === 412) {
      warning(`You are overwriting a helper file with the name of ${fileName}

      Bear this in mind as it may result in unexpected behavior
      `);
    }

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
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
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

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
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
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
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
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
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
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
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

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
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
  createFile,
  renameFile,
  deleteFile,
  uploadFile,
  createFolder,
  renameFolder,
  deleteFolder,
  exitProject,
};

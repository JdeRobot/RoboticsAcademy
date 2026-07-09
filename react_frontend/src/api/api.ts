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
  projectId?: string,
): Promise<Omit<ExerciseData, "worlds">> => {
  if (!projectId) throw new Error("Current Project ID is not set");

  const apiUrl = `/academy/enter_exercise/?project_id=${projectId}`;

  try {
    const response = await axios.get(apiUrl);
    const data = response.data.info;
    data["exercise_id"] = projectId;

    return data;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message, { cause: e });
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
    throw Error(error.response?.data.message, { cause: e });
  }
};

const getExerciseList = async (): Promise<Exercise[]> => {
  const apiUrl = `/academy/get_exercise_list/`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.exercises;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message, { cause: e });
  }
};

const listWorlds = async (project: string) => {
  if (!project) throw new Error("Current Project id is not set");

  const apiUrl = `/academy/get_worlds_list?project=${encodeURIComponent(
    project,
  )}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.worlds_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message, { cause: e });
  }
};

const getRoboticsBackendWorld = async (project: string, world: string) => {
  if (!project) throw new Error("Current Project id is not set");

  const apiUrl = `/academy/get_docker_world_data?world=${encodeURIComponent(
    world,
  )}&project=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);
    return {
      scene: response.data.world.scene,
      robot: response.data.world.robot,
      tools: response.data.world.tools,
      tools_config: response.data.world.tools_config,
    };
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message, { cause: e });
  }
};

const getHelperFile = async (
  project: string,
  language: string,
  fileName: string,
  binary?: boolean,
) => {
  if (!project) throw new Error("Current Project id is not set");
  if (!language) throw new Error("Current Language is not set");
  if (!fileName) throw new Error("File name is not set");

  let apiUrl = `/academy/get_helper_file?project=${encodeURIComponent(
    project,
  )}&language=${encodeURIComponent(language)}&filename=${encodeURIComponent(
    fileName,
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
    throw Error(error.response?.data.message, { cause: e });
  }
};

const getHelperFileList = async (
  project: string,
  language: string,
): Promise<Entry[]> => {
  if (!project) throw new Error("Current Project id is not set");
  if (!language) throw new Error("Current Language is not set");

  const apiUrl = `/academy/get_helper_file_list?project=${encodeURIComponent(
    project,
  )}&language=${encodeURIComponent(language)}`;

  try {
    const response = await axios.get(apiUrl);
    return JSON.parse(response.data.file_list);
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message, { cause: e });
  }
};

const getFileList = async (project: string, user?: string) => {
  if (!project) throw new Error("Current Project id is not set");

  // TODO:add whitelist parameter

  let apiUrl = `/academy/get_file_list?project=${encodeURIComponent(project)}`;

  if (user !== undefined) apiUrl += `&user=${encodeURIComponent(user)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.file_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message, { cause: e });
  }
};

const getFile = async (
  project: string,
  fileName: string,
  user?: string,
  binary?: boolean,
) => {
  if (!project) throw new Error("Project name is not set");
  if (!fileName) throw new Error("File name is not set");

  let apiUrl = `/academy/get_file?project=${encodeURIComponent(
    project,
  )}&filename=${encodeURIComponent(fileName)}`;

  if (user !== undefined) apiUrl += `&user=${encodeURIComponent(user)}`;
  if (binary) apiUrl += `&binary=true`;

  try {
    const response = await axios.get(apiUrl);
    if (binary) {
      return atob(response.data.content);
    }
    return response.data.content;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message, { cause: e });
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
    throw Error(error.response?.data.message, { cause: e });
  }
};

const createFile = async (
  projectId: string,
  fileName: string,
  location: string,
  template?: string,
  warning?: (msg: string) => void,
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

    throw Error(error.response?.data.message, { cause: e });
  }
};

const renameFile = async (
  projectId: string,
  path: string,
  new_path: string,
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
    throw Error(error.response?.data.message, { cause: e });
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
    throw Error(error.response?.data.message, { cause: e });
  }
};

const uploadFile = async (
  projectId: string,
  fileName: string,
  location: string,
  content: string,
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
    throw Error(error.response?.data.message, { cause: e });
  }
};

const createFolder = async (
  projectId: string,
  location: string,
  folderName: string,
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
    throw Error(error.response?.data.message, { cause: e });
  }
};

const renameFolder = async (
  projectId: string,
  path: string,
  new_path: string,
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
    throw Error(error.response?.data.message, { cause: e });
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
    throw Error(error.response?.data.message, { cause: e });
  }
};

export {
  getProjectData,
  getHelperFileList,
  getHelperFile,
  getExerciseList,
  getRoboticsBackendWorld,
  listWorlds,
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

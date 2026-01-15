import axios, { AxiosResponse } from "axios";
import { Exercise } from "src/types/exercises";

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

const getProjectExtraFiles = async (project: string, language: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!language) throw new Error("Current Language is not set");

  const apiUrl = "/academy/user_code_zip/";
  const response = await axios.post(
    apiUrl,
    {
      project: project,
      language: language,
    },
    axiosExtra
  );

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to create app."); // Response error
  }

  return response.data.files;
};

const listUniverses = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

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
  if (!project) throw new Error("Current Project name is not set");

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

export {
  getProjectExtraFiles,
  listUniverses,
  getRoboticsBackendUniverse,
};

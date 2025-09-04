import axios, { AxiosResponse } from "axios";

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
    //@ts-ignore Needed for compatibility with Unibotics
    "X-CSRFToken": csrfToken,
  },
};

const getProjectExtraFiles = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = "/exercises/user_code_zip/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        project: project,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create app."); // Response error
    }

    return response.data.files;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const listUniverses = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = `/exercises/get_universes_list?project=${encodeURIComponent(
    project
  )}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get universes.");
    }

    return response.data.universes_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const listTools = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = `/exercises/get_tools_list?project=${encodeURIComponent(
    project
  )}`;

  try {
    const response = await axios.get(apiUrl);

    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get tools.");
    }

    return response.data.tools_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getRoboticsBackendUniverse = async (
  project: string,
  universe: string
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!universe) throw new Error("Universe name is not set");

  const apiUrl = `/exercises/get_docker_universe_data?universe=${encodeURIComponent(
    universe
  )}&project=${encodeURIComponent(project)}`;

  try {
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
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

export {
  getProjectExtraFiles,
  listUniverses,
  getRoboticsBackendUniverse,
  listTools,
};

import { useState, useEffect, useRef } from "react";
import { useUnload } from "../../hooks/useUnload";
import { CommsManager } from "jderobot-commsmanager";
import React from "react";

import IdeInterface, {
  Entry,
  ExtraApi,
  StatusBarComponents,
  ViewersEntry,
  VncViewer,
} from "jderobot-ide-interface";
import { ExerciseProvider } from "Contexts/ExerciseContext";
import { ExerciseHeader } from "Components/headers";
import { getRoboticsBackendUniverse, listUniverses } from "Helpers/api";
import Frequencies from "Components/statusBar/Frequencies";
import CameraAltRoundedIcon from "@mui/icons-material/CameraAltRounded";
import Camera from "Components/visualizers/Camera";
import TerminalRoundedIcon from "@mui/icons-material/TerminalRounded";
import ImportantDevicesRoundedIcon from "@mui/icons-material/ImportantDevicesRounded";
import VideoCameraBackRoundedIcon from "@mui/icons-material/VideoCameraBackRounded";
import { StyledExerciseContainer } from "Styles/layouts/ExerciseContainer.styles";
import PrecisionManufacturingRoundedIcon from "@mui/icons-material/PrecisionManufacturingRounded";
import {
  defaultCppCode,
  defaultPythonCode,
} from "Components/../constants/code";

const base_file_python = {
  name: "academy.py",
  is_dir: false,
  path: "academy.py",
  group: "code",
  access: true,
  files: [],
};

const base_file_cpp = {
  name: "academy.cpp",
  is_dir: false,
  path: "academy.cpp",
  group: "code",
  access: true,
  files: [],
};

const ExerciseContainer = ({
  project,
  tools,
  url,
  hasDLModel,
  children,
}: {
  project: string;
  tools: string[];
  url?: string;
  hasDLModel: boolean;
  children: JSX.Element;
}) => {
  const [manager, setManager] = useState<CommsManager | null>(null);
  const [universes, setUniverses] = useState<string[] | undefined>(undefined);
  const [showSim, setSimVisible] = useState<boolean>(true);
  const [showWebGUI, setWebGUIVisible] = useState<boolean>(true);
  const [showCamera, setCameraVisible] = useState<boolean>(true);
  const [showRviz, setRvizVisible] = useState<boolean>(true);
  const [showTerminal, setTerminalVisible] = useState<boolean>(true);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both"
  );

  const [language, setLanguage] = useState<string>("python");
  const [baseFile, setBaseFile] = useState<Entry>(base_file_cpp);
  const [code, _setCode] = useState<string>(defaultPythonCode);
  const codeRef = useRef<string>(defaultPythonCode);

  const setCode = (data: string) => {
    codeRef.current = data;
    _setCode(data);
  };

  const getUniverseList = async (project: string) => {
    const list = await listUniverses(project);
    if (list.length === 0) {
      list.push("");
    }

    setUniverses(list);
  };

  const toolsList: ViewersEntry[] = [];

  if (tools.includes("web_gui")) {
    toolsList.push({
      component: children,
      icon: <ImportantDevicesRoundedIcon />,
      name: "Web Gui",
      active: showWebGUI,
      activate: setWebGUIVisible,
    });
  }

  if (tools.includes("webcam")) {
    toolsList.push({
      component: <Camera />,
      icon: <CameraAltRoundedIcon />,
      name: "WebCam",
      active: showCamera,
      activate: setCameraVisible,
    });
  }

  if (tools.includes("simulator")) {
    toolsList.push({
      component: <VncViewer commsManager={manager} port={6080} />,
      icon: <VideoCameraBackRoundedIcon />,
      name: "Gazebo",
      active: showSim,
      activate: setSimVisible,
    });
  }

  if (tools.includes("rviz")) {
    toolsList.push({
      component: <VncViewer commsManager={manager} port={6081} />,
      icon: <PrecisionManufacturingRoundedIcon />,
      name: "Rviz",
      active: showRviz,
      activate: setRvizVisible,
    });
  }

  if (tools.includes("console")) {
    toolsList.push({
      component: <VncViewer commsManager={manager} port={1108} />,
      icon: <TerminalRoundedIcon />,
      name: "Terminal",
      active: showTerminal,
      activate: setTerminalVisible,
    });
  }

  // RB manager setup
  const connected = useRef<boolean>(false);

  useEffect(() => {
    const manager = CommsManager.getInstance();
    setManager(manager);
  }, []);

  const connectWithRetry = async (
    desiredState?: string,
    callback?: () => void
  ) => {
    if (!manager || connected.current) {
      return;
    }
    try {
      const currManager = CommsManager.getInstance();
      await currManager.connect();
      getUniverseList(project);
      console.log("Connected!", currManager.getState());
      connected.current = true;
      setManager(currManager);
      if (callback) {
        waitManagerState(desiredState ? desiredState : "connected", callback);
      }
    } catch (e: unknown) {
      console.log("Connection failed, trying again!");
      setTimeout(connectWithRetry, 2000);
    }
  };

  const waitManagerState = async (state: string, callback: any) => {
    if (manager?.getState() === state) {
      callback();
    } else {
      return setTimeout(waitManagerState, 100, state, callback);
    }
  };

  useUnload(() => {
    if (manager) {
      manager.disconnect();
      connected.current = false;
    }
  });

  useEffect(() => {
    if (language === "cpp") {
      setBaseFile(base_file_cpp);
    } else {
      setBaseFile(base_file_python);
    }
  }, [language]);

  const editorApi: ExtraApi = {
    file: {
      get: (project: string, file: Entry) => {
        const func = async (file: Entry) => {
          if (file.name === "academy.cpp") {
            return defaultCppCode;
          } else {
            return defaultPythonCode;
          }
        };

        return func(file);
      },
      save: (project: string, file: Entry, content: string) => {
        console.log("saveFile", content);
        setCode(content);
        return saveFile(project, file.path, content);
      },
    },
    universes: {
      list: (project: string) => {
        return listUniverses(project);
      },
      get_config: async (project: string, universe: string) => {
        return getRoboticsBackendUniverse(project, universe);
      },
    },
  };

  const statusBar: StatusBarComponents = {
    extras: [<Frequencies manager={manager} />],
  };

  return (
    <StyledExerciseContainer>
      <ExerciseProvider manager={manager} code={codeRef.current}>
        <ExerciseHeader
          project={project}
          language={language}
          setLanguage={setLanguage}
          url={url}
          setLayout={setLayout}
          hasDLModel={hasDLModel}
          connectManager={connectWithRetry}
        />
        <IdeInterface
          commsManager={manager}
          resetManager={connectWithRetry}
          project={project}
          api={editorApi}
          viewers={toolsList}
          options={{ editor: { onlyOneFile: true, notShowSave: true } }}
          layout={layout}
          statusBarComponents={statusBar}
          explorers={[]}
          extraEditors={[]}
          baseFile={baseFile}
          baseUniverse={universes ? universes[0] : undefined}
        />
      </ExerciseProvider>
    </StyledExerciseContainer>
  );
};

export default ExerciseContainer;

function saveFile(
  project: string,
  path: string,
  content: string
): Promise<void> {
  const func = async () => {
    return;
  };

  return func();
}

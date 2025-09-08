import { useState, useEffect, useRef } from "react";
import { useUnload } from "../../hooks/useUnload";
import { CommsManager } from "jderobot-commsmanager";

import IdeInterface, {
  Entry,
  ErrorProvider,
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
import { AcademyThemeProvider } from "Contexts/AcademyThemeContext";

const defaultCode = `import WebGUI
import HAL
import Frequency
# Enter sequential code!

while True:
    # Enter iterative code!
    Frequency.tick()
`;

const base_file = {
  name: "academy.py",
  is_dir: false,
  path: "academy.py",
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
  const [showTerminal, setTerminalVisible] = useState<boolean>(true);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both"
  );

  const [code, _setCode] = useState<string>(defaultCode);
  const codeRef = useRef<string>(defaultCode);

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

  var toolsList: ViewersEntry[] = [];

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

  const resetManager = () => {
    CommsManager.deleteInstance();
    const manager = CommsManager.getInstance();
    setManager(manager);
  };

  const connectWithRetry = async () => {
    if (!manager || connected.current) {
      return;
    }
    try {
      await manager.connect();
      getUniverseList(project);
      console.log("Connected!", manager.getState());
      connected.current = true;
    } catch (error) {
      console.log("Connection failed, trying again!");
      setTimeout(connectWithRetry, 1000);
    }
  };

  useEffect(() => {
    if (manager) {
      console.log("The manager is up!");
      connectWithRetry();
    }
  }, [manager]);

  useUnload(() => {
    if (manager) {
      manager.disconnect();
      connected.current = false;
    }
  });

  const editorApi: ExtraApi = {
    file: {
      get: (project: string, file: Entry) => {
        const func = async () => {
          console.log(codeRef.current);
          return codeRef.current;
        };

        return func();
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
          manager={manager}
          url={url}
          setLayout={setLayout}
          hasDLModel={hasDLModel}
        />
        <IdeInterface
          commsManager={manager}
          resetManager={resetManager}
          project={project}
          api={editorApi}
          viewers={toolsList}
          options={{ editor: { onlyOneFile: true, notShowSave: true } }}
          layout={layout}
          statusBarComponents={statusBar}
          explorers={[]}
          extraEditors={[]}
          baseFile={base_file}
          baseUniverse={universes ? universes[0] : undefined}
        />
      </ExerciseProvider>
    </StyledExerciseContainer>
  );
};

export default ExerciseContainer;

function saveFile(project: string, path: any, content: string): Promise<void> {
  const func = async () => {
    return;
  };

  return func();
}

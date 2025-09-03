import React, {
  useState,
  useEffect,
  useRef,
  Component,
} from "react";
import axios, { AxiosResponse } from "axios";
import { useUnload } from "./../../hooks/useUnload";
// import HeaderMenu from "./components/HeaderMenu";
// import { SimulatorIcon, TerminalIcon } from "./components/icons";
import { CommsManager } from "jderobot-commsmanager";
import "../../styles/wrappers/ExerciseContainer.css";

import IdeInterface, {
  Entry,
  ErrorProvider,
  ExtraApi,
  StatusBarComponents,
  Theme,
  ThemeProvider,
  VncViewer,
} from "jderobot-ide-interface";
import { ExerciseProvider } from "Contexts/ExerciseContext";
import HeaderMenu from "Components/HeaderMenu";
import { getRoboticsBackendUniverse, listUniverses } from "Helpers/api";
import { SimulatorIcon, TerminalIcon } from "Icons/index";
import Frequencies from "Components/statusBar/Frequencies";

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
  url,
  children,
}: {
  project: string;
  url?: string;
  children: Component;
}) => {
  const [manager, setManager] = useState<CommsManager | null>(null);
  const [universes, setUniverses] = useState<string[] | undefined>(undefined);
  const [showSim, setSimVisible] = useState<boolean>(true);
  const [showMonitor, setMonitorVisible] = useState<boolean>(true);
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
    const list = await listUniverses(project)
    setUniverses(list);
  };

  const uploadCode = async (code: string) => {
    setCode(code);
  };


  // RB manager setup
  const connected = useRef<boolean>(false);

  useEffect(() => {
    setManager(manager);
    try {
      getUniverseList(project);
    } catch (error) {
      setUniverses(undefined);
    }
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

  useUnload((event: any) => {
    if (manager) {
      manager.disconnect();
      connected.current = false;
    }
  });

  // Get available tools from the db
  const treeMonitor = {
    component: children,
    icon: <SimulatorIcon />,
    name: "Web Gui",
    active: showMonitor,
    activate: setMonitorVisible,
  };

  const gazeboViewer = {
    component: <VncViewer commsManager={manager} port={6080} />,
    icon: <SimulatorIcon />,
    name: "Gazebo",
    active: showSim,
    activate: setSimVisible,
  };

  const terminalViewer = {
    component: <VncViewer commsManager={manager} port={1108} />,
    icon: <TerminalIcon />,
    name: "Terminal",
    active: showTerminal,
    activate: setTerminalVisible,
  };

  const editorApi: ExtraApi = {
    file: {
      get: (project: string, file: Entry) => {
        const func = async () => {
          console.log(codeRef.current)
          return codeRef.current;
        };

        return func();
      },
      save: (project: string, file: Entry, content: string) => {
        console.log("saveFile", content)
        setCode(content)
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
    extras: [<Frequencies manager={manager}/>],
  };

  const darkTheme: Theme = {
    palette: {
      darkText: "#ededf2",
      text: "#000000",
      placeholderText: "#a6a6bf",
      success: "#29ac29",
      warning: "#af5500ff",
      error: "#802626",
      background: "#16161d",
      primary: "#ffa726",
      secondary: "#ff8800",
      scrollbar: "#6f6f90",
      border: {
        warning: "#af5500ff",
        error: "#772222",
        info: "#134f53",
      },
      progressBar: {
        background: "#134f53",
        color: "#1d777c",
      },
      button: {
        error: "#9e2e2e",
        success: "#29ac29",
        warning: "#af5500ff",
        info: "#134f53",
        hoverError: "#c63939",
        hoverSuccess: "#29ac29",
        hoverWarning: "#e05a00ffff",
        hoverInfo: "#1d777c",
      },
      selectedGradient:
        "linear-gradient( -45deg, #12494c 0%, #584f42 50%, #909c7b 100%)",
    },
    roundness: 5,
    monacoTheme: "dark",
  };

  return (
    <ErrorProvider>
      <ThemeProvider theme={darkTheme}>
        <div className="exercise-container" style={{ display: "flex" }}>
          <ExerciseProvider
            manager={manager}
            code={codeRef.current}
          >
            <HeaderMenu
              project={project}
              manager={manager}
              url={url}
              uploadCode={uploadCode}
              setLayout={setLayout}
            />
            <IdeInterface
              commsManager={manager}
              resetManager={resetManager}
              project={project}
              api={editorApi}
              viewers={[treeMonitor, gazeboViewer, terminalViewer]}
              options={[]}
              layout={layout}
              statusBarComponents={statusBar}
              explorers={[]}
              extraEditors={[]}
              baseFile={base_file}
              baseUniverse={universes ? universes[0] : undefined}
            />
          </ExerciseProvider>
        </div>
      </ThemeProvider>
    </ErrorProvider>
  );
};

export default ExerciseContainer;

function saveFile(project: string, path: any, content: string): Promise<void> {
  const func = async () => {
    return;
  };
  
  return func();
}
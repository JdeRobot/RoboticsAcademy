import React from "react";
import { useState, useEffect, useRef, JSX } from "react";
import { CommsManager, states } from "jderobot-commsmanager";

import IdeInterface, {
  Entry,
  ExtraApi,
  ExtraSnippets,
  StatusBarComponents,
  useError,
} from "jderobot-ide-interface";
import { ExerciseProvider } from "Contexts/ExerciseContext";
import { ExerciseHeader } from "Components/headers";
import { getFile, getRoboticsBackendWorld, listWorlds, saveFile } from "Api";
import Frequencies from "Components/statusBar/Frequencies";
import { StyledExerciseContainer } from "Styles/layouts/ExerciseContainer.styles";
import { getHalGuiMethods } from "Helpers/editor";
import { clearTimeouts, subscribe, unsubscribe } from "Helpers/utils";
import { fileExplorer } from "Helpers/explorer";
import getTools from "Helpers/tools";

const base_file = {
  name: `academy.py`,
  is_dir: false,
  path: `academy.py`,
  group: "code",
  access: true,
  files: [],
};

const ExerciseContainer = ({
  project,
  name,
  multiLanguage,
  tools,
  url,
  children,
}: {
  project: string;
  name: string;
  tools: string[];
  url?: string;
  multiLanguage: boolean;
  children: JSX.Element;
}) => {
  const { warning } = useError();
  const hasTriedToConnect = useRef(false);
  const timeoutRef = useRef<number | null>(null);
  const connectTimeoutRef = useRef<number | null>(null);
  const [manager, setManager] = useState<CommsManager | null>(null);
  const [worlds, setWorlds] = useState<string[] | undefined>(undefined);
  const toolsList = getTools(manager, tools, children);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both",
  );
  const userRef = useRef<string | undefined>(undefined);
  const addressRef = useRef<string>(`ws://127.0.0.1:7163`);

  const getWorldList = async (project: string) => {
    const list = await listWorlds(project);
    if (list.length === 0) {
      list.push("");
    }

    setWorlds(list);
  };

  const resetWorld = (e: unknown) => {
    const T = CustomEvent<{ detail: unknown }>;
    if (e instanceof T) {
      if (e.detail.state == states.IDLE) {
        setWorlds(undefined);
      }
    }
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", resetWorld);

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});

      if (hasTriedToConnect.current) {
        const currManager = CommsManager.getInstance();
        if (currManager) {
          currManager.disconnect();
          CommsManager.deleteInstance();
          setManager(null);
        }
      }

      clearTimeouts([timeoutRef, connectTimeoutRef]);
    };
  }, []);

  const connectWithRetry = async (
    desiredState?: string,
    callback?: () => void,
  ) => {
    try {
      const currManager = CommsManager.getInstance(addressRef.current);
      hasTriedToConnect.current = true;
      await currManager.connect();
      getWorldList(project);
      console.log("Connected!", currManager.getState());
      setManager(currManager);
      if (callback) {
        waitManagerState(desiredState ? desiredState : "connected", callback);
      }
    } catch {
      console.log("Connection failed, trying again!");
      timeoutRef.current = window.setTimeout(
        connectWithRetry,
        2000,
        desiredState,
        callback,
      );
    }
  };

  const waitManagerState = async (state: string, callback: () => void) => {
    const currManager = CommsManager.getInstance(addressRef.current);
    if (currManager?.getState() === state) {
      callback();
    } else {
      connectTimeoutRef.current = window.setTimeout(
        waitManagerState,
        100,
        state,
        callback,
      );
    }
  };

  const editorApi: ExtraApi = {
    file: {
      get: (project: string, file: Entry) => {
        //TODO: allow binary support
        return getFile(project, file.path);
      },
      save: (project: string, file: Entry, content: string) => {
        return saveFile(project, file.path, content);
      },
    },
    worlds: {
      list: (project: string) => {
        return listWorlds(project);
      },
      get_config: async (project: string, world: string) => {
        return getRoboticsBackendWorld(project, world);
      },
    },
  };

  const statusBar: StatusBarComponents = {
    extras: [<Frequencies key="freq-statusBar" manager={manager} />],
  };

  const extraSnippets: ExtraSnippets = {
    triggers: ["WebGUI", "HAL", "Frequencies"],
    loader: (prevWord: string) => {
      return getHalGuiMethods(prevWord);
    },
  };

  return (
    <StyledExerciseContainer>
      <ExerciseProvider manager={manager}>
        <ExerciseHeader
          project={project}
          name={name}
          supportedLanguages={multiLanguage ? ["python", "cpp"] : ["python"]}
          url={url}
          setLayout={setLayout}
          commsManager={manager}
          connectManager={connectWithRetry}
          userRef={userRef}
        />
        <IdeInterface
          commsManager={manager}
          project={project}
          api={editorApi}
          viewers={toolsList}
          options={[]}
          layout={layout}
          statusBarComponents={statusBar}
          explorers={[fileExplorer(warning)]}
          extraEditors={[]}
          baseFile={base_file}
          baseWorld={worlds ? worlds[0] : undefined}
          extraSnippets={extraSnippets}
        />
      </ExerciseProvider>
    </StyledExerciseContainer>
  );
};

export default ExerciseContainer;

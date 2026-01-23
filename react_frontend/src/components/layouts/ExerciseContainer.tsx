import React from "react";
import { useState, useEffect, useRef } from "react";
import { CommsManager, states } from "jderobot-commsmanager";

import IdeInterface, {
  Entry,
  ExtraApi,
  ExtraSnippets,
  StatusBarComponents,
} from "jderobot-ide-interface";
import { ExerciseProvider } from "Contexts/ExerciseContext";
import { ExerciseHeader } from "Components/headers";
import {
  getFile,
  getRoboticsBackendUniverse,
  listUniverses,
  saveFile,
} from "Api";
import Frequencies from "Components/statusBar/Frequencies";
import { StyledExerciseContainer } from "Styles/layouts/ExerciseContainer.styles";
import { getHalGuiMethods } from "Helpers/editor";
import { subscribe, unsubscribe } from "Helpers/utils";
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
  const [manager, setManager] = useState<CommsManager | null>(null);
  const [universes, setUniverses] = useState<string[] | undefined>(undefined);
  const toolsList = getTools(manager, tools, children);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both"
  );
  const [language, setLanguage] = useState<string>("python");

  const getUniverseList = async (project: string) => {
    const list = await listUniverses(project);
    if (list.length === 0) {
      list.push("");
    }

    setUniverses(list);
  };

  // RB manager setup
  const connected = useRef<boolean>(false);

  const resetUniverse = (e: unknown) => {
    const T = CustomEvent<{ detail: unknown }>;
    if (e instanceof T) {
      if (e.detail.state == states.IDLE) {
        setUniverses(undefined);
      }
    }
  };

  useEffect(() => {
    const manager = CommsManager.getInstance();
    setManager(manager);
    subscribe("CommsManagerStateChange", resetUniverse);

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});
      const currManager = CommsManager.getInstance();
      if (currManager) {
        currManager.disconnect();
      }
    };
  }, []);

  const connectWithRetry = async (
    desiredState?: string,
    callback?: () => void
  ) => {
    console.log(manager?.getState(), CommsManager.getInstance().getState());
    if (!manager || manager?.getState() != "idle") {
      return;
    }
    try {
      const currManager = CommsManager.getInstance();
      console.log(currManager);
      await currManager.connect();
      getUniverseList(project);
      console.log("Connected!", currManager.getState());
      connected.current = true;
      setManager(currManager);
      if (callback) {
        waitManagerState(desiredState ? desiredState : "connected", callback);
      }
    } catch {
      console.log("Connection failed, trying again!");
      setTimeout(connectWithRetry, 2000, desiredState, callback);
    }
  };

  const waitManagerState = async (state: string, callback: () => void) => {
    if (manager?.getState() === state) {
      callback();
    } else {
      return setTimeout(waitManagerState, 100, state, callback);
    }
  };

  const editorApi: ExtraApi = {
    file: {
      get: (project: string, file: Entry) => {
        //TODO: allow binary support
        console.log(file);
        return getFile(project, file.path);
      },
      save: (project: string, file: Entry, content: string) => {
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
          language={multiLanguage ? language : undefined}
          setLanguage={setLanguage}
          url={url}
          setLayout={setLayout}
          connectManager={connectWithRetry}
        />
        <IdeInterface
          commsManager={manager}
          connectManager={connectWithRetry}
          project={project}
          api={editorApi}
          viewers={toolsList}
          options={[]}
          layout={layout}
          statusBarComponents={statusBar}
          explorers={[fileExplorer]}
          extraEditors={[]}
          baseFile={base_file}
          baseUniverse={universes ? universes[0] : undefined}
          extraSnippets={extraSnippets}
        />
      </ExerciseProvider>
    </StyledExerciseContainer>
  );
};

export default ExerciseContainer;

import React from "react";
import { useState, useEffect, useRef } from "react";
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
import {
  getFile,
  getRoboticsBackendUniverse,
  listUniverses,
  saveFile,
} from "Api";
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
  const [universes, setUniverses] = useState<string[] | undefined>(undefined);
  const toolsList = getTools(manager, tools, children);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both"
  );

  const getUniverseList = async (project: string) => {
    const list = await listUniverses(project);
    if (list.length === 0) {
      list.push("");
    }

    setUniverses(list);
  };

  const resetUniverse = (e: unknown) => {
    const T = CustomEvent<{ detail: unknown }>;
    if (e instanceof T) {
      if (e.detail.state == states.IDLE) {
        setUniverses(undefined);
      }
    }
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", resetUniverse);

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
    callback?: () => void
  ) => {
    try {
      const currManager = CommsManager.getInstance();
      hasTriedToConnect.current = true;
      await currManager.connect();
      getUniverseList(project);
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
        callback
      );
    }
  };

  const waitManagerState = async (state: string, callback: () => void) => {
    const currManager = CommsManager.getInstance();
    if (currManager?.getState() === state) {
      callback();
    } else {
      connectTimeoutRef.current = window.setTimeout(
        waitManagerState,
        100,
        state,
        callback
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
          supportedLanguages={multiLanguage ? ["python", "cpp"] : ["python"]}
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
          explorers={[fileExplorer(warning)]}
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

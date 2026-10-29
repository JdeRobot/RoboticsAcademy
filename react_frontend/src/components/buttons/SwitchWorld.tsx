/* eslint-disable react/react-in-jsx-scope */
import PublicRoundedIcon from '@mui/icons-material/PublicRounded';
import { getRoboticsBackendWorld, listWorlds } from "Api";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { subscribe, unsubscribe } from 'Helpers/utils';
import { CommsManager, states } from "jderobot-commsmanager";
import { useError } from "jderobot-ide-interface";
import { useEffect, useState } from "react";
import { Dropdown } from "./Dropdown";

const TerminateUniverseButton = ({ project, commsManager, baseWorld }: {
  project: string, commsManager: CommsManager | null,
  baseWorld?: string;

}) => {
  const theme = useAcademyTheme();
  const { warning, error } = useError();
  const [world, setWorld] = useState<string | undefined>(
    commsManager?.getWorld(),
  );
  const [worldList, setWorldList] = useState<string[]>([]);
  const wrnMsg =
    "Failed to connect with the Robotics Backend. Please make sure it is connected.";

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const resetWorld = (e: any) => {
    if (e.detail.state == states.IDLE) {
      setWorld(baseWorld);
    }
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", resetWorld);

    return () => {
      unsubscribe("CommsManagerStateChange", () => { });
    };
  }, []);

  useEffect(() => {
    if (commsManager) {
      console.log("Change World", commsManager.getWorld());
      setWorld(commsManager.getWorld());
    }
  }, [commsManager?.getWorld()]);

  useEffect(() => {
    const get_world_list = async () => {
      const list = await listWorlds(project);
      setWorldList(list);
    };
    get_world_list();
  }, [project]);

  useEffect(() => {
    if (baseWorld !== undefined) {
      selectWorld(baseWorld);
    }
  }, [baseWorld]);

  const terminateWorld = async () => {
    const manager = CommsManager.getInstance();

    if (!manager) {
      warning(wrnMsg);
      return;
    }

    // Down the RB ladder
    await manager.terminateApplication();
    await manager.terminateTools();
    await manager.terminateWorld();
  };

  const launchWorld = async (world: string) => {
    const manager = CommsManager.getInstance();

    if (!manager) {
      warning(wrnMsg);
      return;
    }

    if (project === "") {
      error("Failed to find the current project name.");
      return;
    }

    const worldConfig = await getRoboticsBackendWorld(project, world);

    const tools = worldConfig.tools;
    const scene_config = worldConfig.scene;
    const robot_config = worldConfig.robot;

    const world_config = {
      name: world,
      scene: scene_config,
      robot: robot_config,
    };

    await manager.launchWorld(world_config);
    console.log("RB world launched!");
    await manager.prepareTools(tools, worldConfig.tools_config);
    console.log("Tools ready!");
  };

  const selectWorld = async (worldName: string) => {
    console.log(worldName);

    try {
      // Launch if new world selected
      if (worldName !== world) {
        if (world) await listWorlds(project);
        if (world) await terminateWorld();
        await launchWorld(worldName);
        console.log("Launch world successful");
      }
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Unable to retrieve world config: " + e.message);
        error("Unable to retrieve world config: " + e.message);
      }
    }
  };

  if (worldList.length === 0) {
    return <></>
  }

  return (
    <Dropdown
      id="world"
      title="Switch world"
      width={360}
      setter={selectWorld}
      possibleValues={worldList}
    >
      <PublicRoundedIcon htmlColor={theme.palette.text} />
    </Dropdown>
  );
};

export default TerminateUniverseButton;


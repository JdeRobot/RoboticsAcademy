import { CommsManager } from "jderobot-commsmanager";
import { ViewersEntry, VncViewer } from "jderobot-ide-interface";
import { JSX, useEffect, useState } from "react";

import CameraAltRoundedIcon from "@mui/icons-material/CameraAltRounded";
import ImportantDevicesRoundedIcon from "@mui/icons-material/ImportantDevicesRounded";
import OndemandVideoRoundedIcon from "@mui/icons-material/OndemandVideoRounded";
import PrecisionManufacturingRoundedIcon from "@mui/icons-material/PrecisionManufacturingRounded";
import TerminalRoundedIcon from "@mui/icons-material/TerminalRounded";
import VideoCameraBackRoundedIcon from "@mui/icons-material/VideoCameraBackRounded";
import Camera from "Components/visualizers/Camera";
import Video from "Components/visualizers/Video";

const getTools = (
  manager: CommsManager | null,
  tools: string[],
  children: JSX.Element,
) => {
  const [showSim, setSimVisible] = useState<boolean>(true);
  const [showWebGUI, setWebGUIVisible] = useState<boolean>(true);
  const [showCamera, setCameraVisible] = useState<boolean>(true);
  const [showVideo, setVideoVisible] = useState<boolean>(true);
  const [showRviz, setRvizVisible] = useState<boolean>(true);
  const [showTerminal, setTerminalVisible] = useState<boolean>(true);
  const [toolsList, setToolsList] = useState<ViewersEntry[]>([]);

  useEffect(() => {
    const list: ViewersEntry[] = [];
    if (tools.includes("web_gui")) {
      list.push({
        component: children,
        icon: <ImportantDevicesRoundedIcon />,
        name: "Web Gui",
        group: "debug-interface",
        active: showWebGUI,
        activate: setWebGUIVisible,
      });
    }

    if (tools.includes("webcam")) {
      list.push({
        component: <Camera visible={showCamera} />,
        icon: <CameraAltRoundedIcon />,
        name: "WebCam",
        group: "video-input",
        active: showCamera,
        activate: setCameraVisible,
      });
    }

    if (tools.includes("video")) {
      list.push({
        component: <Video visible={showVideo} />,
        icon: <OndemandVideoRoundedIcon />,
        name: "Local video",
        group: "video-input",
        active: showVideo,
        activate: setVideoVisible,
      });
    }

    if (tools.includes("simulator")) {
      list.push({
        component: (
          <VncViewer
            commsManager={manager}
            port={"7163/sim"}
            message={"Click Play to connect to the Robotics Backend"}
          />
        ),
        icon: <VideoCameraBackRoundedIcon />,
        name: "Gazebo",
        active: showSim,
        activate: setSimVisible,
      });
    }

    if (tools.includes("rviz")) {
      list.push({
        component: (
          <VncViewer
            commsManager={manager}
            port={"7163/rviz"}
            message={"Click Play to connect to the Robotics Backend"}
          />
        ),
        icon: <PrecisionManufacturingRoundedIcon />,
        name: "Rviz",
        active: showRviz,
        activate: setRvizVisible,
      });
    }

    if (tools.includes("console")) {
      list.push({
        component: (
          <VncViewer
            commsManager={manager}
            port={"7163/console"}
            message={"Click Play to connect to the Robotics Backend"}
          />
        ),
        icon: <TerminalRoundedIcon />,
        name: "Terminal",
        active: showTerminal,
        activate: setTerminalVisible,
      });
    }
    console.log(list)
    setToolsList(list)
  }, [tools]);


  return toolsList;
};

export default getTools;

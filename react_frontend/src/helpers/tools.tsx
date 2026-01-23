import React, { useState } from "react";
import { CommsManager } from "jderobot-commsmanager";
import { ViewersEntry, VncViewer } from "jderobot-ide-interface";

import CameraAltRoundedIcon from "@mui/icons-material/CameraAltRounded";
import Camera from "Components/visualizers/Camera";
import Video from "Components/visualizers/Video";
import TerminalRoundedIcon from "@mui/icons-material/TerminalRounded";
import ImportantDevicesRoundedIcon from "@mui/icons-material/ImportantDevicesRounded";
import VideoCameraBackRoundedIcon from "@mui/icons-material/VideoCameraBackRounded";
import OndemandVideoRoundedIcon from "@mui/icons-material/OndemandVideoRounded";
import PrecisionManufacturingRoundedIcon from "@mui/icons-material/PrecisionManufacturingRounded";

const getTools = (
  manager: CommsManager | null,
  tools: string[],
  children: JSX.Element
) => {
  const [showSim, setSimVisible] = useState<boolean>(true);
  const [showWebGUI, setWebGUIVisible] = useState<boolean>(true);
  const [showCamera, setCameraVisible] = useState<boolean>(true);
  const [showVideo, setVideoVisible] = useState<boolean>(true);
  const [showRviz, setRvizVisible] = useState<boolean>(true);
  const [showTerminal, setTerminalVisible] = useState<boolean>(true);

  const toolsList: ViewersEntry[] = [];

  if (tools.includes("web_gui")) {
    toolsList.push({
      component: children,
      icon: <ImportantDevicesRoundedIcon />,
      name: "Web Gui",
      group: "debug-interface",
      active: showWebGUI,
      activate: setWebGUIVisible,
    });
  }

  if (tools.includes("webcam")) {
    toolsList.push({
      component: <Camera visible={showCamera} />,
      icon: <CameraAltRoundedIcon />,
      name: "WebCam",
      group: "video-input",
      active: showCamera,
      activate: setCameraVisible,
    });
  }

  if (tools.includes("video")) {
    toolsList.push({
      component: <Video visible={showVideo} />,
      icon: <OndemandVideoRoundedIcon />,
      name: "Local video",
      group: "video-input",
      active: showVideo,
      activate: setVideoVisible,
    });
  }

  if (tools.includes("simulator")) {
    toolsList.push({
      component: (
        <VncViewer
          commsManager={manager}
          port={6080}
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
    toolsList.push({
      component: (
        <VncViewer
          commsManager={manager}
          port={6081}
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
    toolsList.push({
      component: (
        <VncViewer
          commsManager={manager}
          port={6082}
          message={"Click Play to connect to the Robotics Backend"}
        />
      ),
      icon: <TerminalRoundedIcon />,
      name: "Terminal",
      active: showTerminal,
      activate: setTerminalVisible,
    });
  }

  return toolsList;
};

export default getTools;

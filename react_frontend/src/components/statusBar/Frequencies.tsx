import { useState, useEffect } from "react";
import { CommsManager, events, ManagerMsg } from "jderobot-commsmanager";
import { StyledStatusBarEntry } from "jderobot-ide-interface";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import React from "react";

type FrequenciesData = {
  brain: number;
  gui: number;
  rtf: number;
  fps: number;
  lat: number;
};

const Frequencies = ({ manager }: { manager: CommsManager | null }) => {
  const theme = useAcademyTheme();
  const [frequencies, setFrequencies] = useState<FrequenciesData>({
    brain: 0,
    gui: 0,
    rtf: -1,
    fps: -1,
    lat: -1,
  });

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const updateCallback = (message: ManagerMsg) => {
      const update = message.data.update;
      if (update.brain) {
        setFrequencies(update);
      }
    };

    manager.subscribe(events.UPDATE, updateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
    };
  }, [manager]);

  return (
    <>
      <StyledStatusBarEntry
        text={theme.palette.text}
        title="Application Frequency"
      >
        <label>{`App: ${frequencies.brain.toFixed(0)} Hz`}</label>
      </StyledStatusBarEntry>
      {/* <StyledStatusBarEntry text={theme.palette.text} title="WebGUI Frequency">
        <label>{`Gui: ${frequencies.gui.toFixed(0)} Hz`}</label>
      </StyledStatusBarEntry> */}
      {frequencies.rtf >= 0 && (
        <StyledStatusBarEntry
          text={theme.palette.text}
          title="Real Time Factor"
        >
          <label>{`RTF: ${frequencies.rtf}`}</label>
        </StyledStatusBarEntry>
      )}
      {frequencies.fps >= 0 && (
        <StyledStatusBarEntry text={theme.palette.text} title="Fps">
          <label>{`Fps: ${
            frequencies.fps < 10
              ? frequencies.fps === 0
                ? `0`
                : `0${frequencies.fps}`
              : frequencies.fps
          }`}</label>
        </StyledStatusBarEntry>
      )}
      {frequencies.lat >= 0 && (
        <StyledStatusBarEntry text={theme.palette.text} title="Latency">
          <label>{`Lat: ${
            frequencies.lat >= 1000
              ? `${(frequencies.lat / 1000).toFixed(0)} s`
              : `${frequencies.lat} ms`
          }`}</label>
        </StyledStatusBarEntry>
      )}
    </>
  );
};

export default Frequencies;

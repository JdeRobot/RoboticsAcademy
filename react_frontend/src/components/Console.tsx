/* eslint-disable react/react-in-jsx-scope */
import { Activity, useEffect, useState } from "react";
import { Terminal as StyledTerminal } from "Styles/TerminalPanels.styles";

import { AttachAddon } from "@xterm/addon-attach";
import { ClipboardAddon } from "@xterm/addon-clipboard";
import { FitAddon } from "@xterm/addon-fit";
import { SearchAddon } from "@xterm/addon-search";
import { WebLinksAddon } from "@xterm/addon-web-links";
import { WebglAddon } from "@xterm/addon-webgl";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { subscribe, unsubscribe } from "Helpers/utils";
import type { CommsManager } from "jderobot-commsmanager";
import { contrastSelector } from "jderobot-ide-interface";
import { useXTerm } from "react-xtermjs";

const enabled = (state?: string): boolean => {
  if (
    state === "tools_ready" ||
    state === "application_running" ||
    state === "paused"
  ) {
    return true;
  }

  return false;
};

const Console = ({ commsManager,visible }: { visible: boolean,
  commsManager: CommsManager | null;

 }) => {
  const th = useAcademyTheme();
  const { instance, ref } = useXTerm();
  const [state, setState] = useState<string | undefined>(
    commsManager?.getState(),
  );
  const updateState = (e: any) => {
    setState(e.detail.state);
  };

  const text = contrastSelector(
    th.palette.text,
    th.palette.darkText,
    th.palette.bg,
  );

  useEffect(() => {
    subscribe("CommsManagerStateChange", updateState);

    return () => {
      unsubscribe("CommsManagerStateChange", () => { });
    };
  }, []);

  useEffect(() => {
    if (! enabled(state)) return;
    if (instance === null) return;
    if (ref.current === null) return;

    console.warn(instance, ref.current);
    const cols = instance.cols;
    const rows = instance.rows;

    const ws = new WebSocket(`ws://127.0.0.1:7163/console`);
    ws.binaryType = "arraybuffer";
    ws.onopen = () => {
      instance.focus();
    };

    // Load the addons
    const attachAddon = new AttachAddon(ws);
    const fitAddon = new FitAddon();
    const webglAddon = new WebglAddon();
    const clipboardAddon = new ClipboardAddon();
    const searchAddon = new SearchAddon();
    const webLinksAddon = new WebLinksAddon();
    webglAddon.onContextLoss((e) => {
      webglAddon.dispose();
    });

    instance?.loadAddon(fitAddon);
    instance?.loadAddon(attachAddon);
    // instance?.loadAddon(webglAddon); // This breaks the style
    instance?.loadAddon(clipboardAddon);
    instance?.loadAddon(searchAddon);
    instance?.loadAddon(webLinksAddon);
    fitAddon.fit();

    const handleResize = () => fitAddon.fit();

    instance.onResize(({ cols, rows }) => {
      const resizeMsg = JSON.stringify({ resize: { cols, rows } });
      const binaryMsg = new TextEncoder().encode(resizeMsg);
      ws.send(binaryMsg);
    });

    const resizeObserver = new ResizeObserver(handleResize);

    resizeObserver.observe(ref.current);
    ref.current.focus()
  }, [ref, instance, state]);

  return (
    <Activity mode={visible ? "visible" : "hidden"}>
      <StyledTerminal
        ref={ref}
        bg={th.palette.bg}
        color={text}
        scroll={th.palette.primary}
        roundness={th.roundness}
      />
    </Activity>
  );
};

export default Console;

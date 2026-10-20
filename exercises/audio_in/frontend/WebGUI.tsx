import { useState, useEffect, useRef } from "react";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { events, ManagerMsg, states } from "jderobot-commsmanager";

const code = `class AudioProcessor extends AudioWorkletProcessor {
    constructor() {
      super();
      this.buffer = new Float32Array(512);
      this.offset = 0;
    }
  
    process(inputs, outputs, parameters) {
      const input = inputs[0];
      if (!input || !input[0]) return true;
      const channelData = input[0]; // 128 muestras
  
      this.buffer.set(channelData, this.offset);
      this.offset += channelData.length;
  
      if (this.offset >= 512) {
        this.port.postMessage(this.buffer.slice());
        this.buffer = new Float32Array(512);
        this.offset = 0;
      }
      
      return true;
    }
  }
  
  registerProcessor("audio-worklet", AudioProcessor);`;

// URL en memoria para que el navegador pueda cargarlo como módulo sin que dé problemas
const getWorkletUrl = () => {
  const blob = new Blob([code], { type: "application/javascript" });
  return URL.createObjectURL(blob);
};

function WebGUI() {
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  const audioStream = useRef<MediaStream | null>(null);
  const audioContext = useRef<AudioContext>(null);
  const audioSource = useRef<MediaStreamAudioSourceNode>(null);
  const audioWorklet = useRef<AudioWorkletNode>(null);

  const [text, setText] = useState<string[]>([""]);
  const [recording, setRecording] = useState<boolean>(false);
  const microphonePaused = useRef<boolean>(false);
  const [isAudioReady, setAudioReady] = useState<boolean>(false);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (isAudioReady) {
      startAudio();
    } else {
      stopAudio();
    }

    return () => {
      stopAudio();
    };
  }, [isAudioReady]);

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const stateCallback = (message: ManagerMsg) => {
      const state = message.data.state;

      if (state === states.TOOLS_READY) {
        setAudioReady(true);
      }
      if (state === states.RUNNING) {
        microphonePaused.current = false;
      } else if (state === states.PAUSED) {
        microphonePaused.current = true;
      }
    };

    const currentState = manager.getState();

    if (currentState === states.TOOLS_READY) {
      setAudioReady(true);
    }

    if (currentState === states.RUNNING) {
      setAudioReady(true);
      microphonePaused.current = false;
    }

    if (currentState === states.PAUSED) {
      setAudioReady(true);
      microphonePaused.current = true;
    }

    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    if (update.text) {
      const texts = JSON.parse(update.text);
      if (texts.length > 0) {
        setText((prev) => [...prev, ...texts]);
      }
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setText([""]);
    }
  };

  const startAudio = async () => {
    audioStream.current = await navigator.mediaDevices.getUserMedia({
      audio: true,
    });
    audioContext.current = new AudioContext({ sampleRate: 16000 });

    const urlCode = getWorkletUrl();
    await audioContext.current.audioWorklet.addModule(urlCode);

    audioSource.current = audioContext.current.createMediaStreamSource(
      audioStream.current,
    );
    // AudioWorletNode - audio crudo
    audioWorklet.current = new AudioWorkletNode(
      audioContext.current,
      "audio-worklet",
    );
    audioSource.current.connect(audioWorklet.current);
    audioWorklet.current.port.onmessage = (ev) => {
      // Codificamos en base64
      const audioData = btoa(
        String.fromCharCode(...Array.from(new Uint8Array(ev.data.buffer))),
      );

      const performance_t = performance.now();
      const time = performance_t.toFixed(5).toString().padStart(20, "0");

      if (manager !== null && !microphonePaused.current) {
        manager.send("gui", `pick${audioData}${time}`);
      }
    };

    setRecording(true);
  };

  const stopAudio = async () => {
    if (manager !== null) {
      manager.send("gui", "audio-stop"); // Vacía la última frase
    }

    if (audioWorklet.current) {
      audioWorklet.current.port.onmessage = null;
      audioWorklet.current.disconnect();
      audioWorklet.current = null;
    }

    if (audioSource.current) {
      audioSource.current.disconnect();
      audioSource.current = null;
    }

    if (audioStream.current) {
      audioStream.current.getTracks().forEach((track) => track.stop());
      audioStream.current = null;
    }

    if (audioContext.current) {
      await audioContext.current.close();
      audioContext.current = null;
    }

    setRecording(false);
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <div style={{ display: "flex", flexDirection: "column", gap: "8px" }}>
        {/* <button disabled={recording} onClick={startAudio}>Grabar audio</button>
        <button disabled={!recording} onClick={stopAudio}>Parar audio</button> */}
        <p>{text.join("\n")}</p>
      </div>
    </WebGUIContainer>
  );
}

export default WebGUI;

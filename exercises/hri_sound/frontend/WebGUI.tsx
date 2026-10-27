import React, { useState, useEffect, useRef } from "react";
import { updatePath, addToPath } from "./helpers/VacuumCleanerHelper";
import houseMapDirty from "./resources/mapgrannyannie_dirty.png";
import Vacuum from "./resources/vacuum.svg";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";

import "./css/GUICanvas.css";

// ---------------------------------------------------------------------
// Audio (audio_out)
// ---------------------------------------------------------------------

// Códigos del AudioWorklet, cargados desde un Blob para que el archivo sea autocontenido.
// Recibe Float32Array (audio a encolar) o "clear" (vaciar la cola).
const WORKLET_CODE = `
  class PCMPlayerProcessor extends AudioWorkletProcessor {
    constructor() {
      super();
      this.queue = [];
      this.offset = 0;
      this.port.onmessage = (event) => {
        if (event.data === "clear") {
          this.queue = [];
          this.offset = 0;
          return;
        }
        this.queue.push(event.data);
      };
    }

    process(_inputs, outputs) {
      const output = outputs[0][0];
      let written = 0;

      while (written < output.length) {
        if (this.queue.length === 0) {
          output[written++] = 0;
          continue;
        }

        const chunk = this.queue[0];
        const available = chunk.length - this.offset;
        const toCopy = Math.min(available, output.length - written);

        output.set(chunk.subarray(this.offset, this.offset + toCopy), written);
        this.offset += toCopy;
        written += toCopy;

        if (this.offset >= chunk.length) {
          this.queue.shift();
          this.offset = 0;
        }
      }

      return true;
    }
  }

  registerProcessor('pcm-player', PCMPlayerProcessor);
`;

const MICROPHONE_WORKLET_CODE = `class AudioProcessor extends AudioWorkletProcessor {
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
  const blob = new Blob([MICROPHONE_WORKLET_CODE], { type: "application/javascript" });
  return URL.createObjectURL(blob);
};

const SAMPLE_RATE = 48000; // debe coincidir con SAMPLE_RATE de audio_source.py
const MICROPHONE_SAMPLE_RATE = 16000;

// Estados que manda el backend -> texto que se muestra
const STATUS_LABELS: Record<string, string> = {
  idle: "Listo",
  synthesizing: "Sintetizando...",
  reading_file: "Leyendo archivo...",
  playing: "Reproduciendo",
  done: "Terminado",
  stopped: "Detenido",
};

// Traduce el estado del backend; si no lo conoce (p. ej. "error: ..."), lo deja tal cual
const statusLabel = (s: string) => STATUS_LABELS[s] ?? s;

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [vacuumPose, setVacuumPose] = useState<number[] | undefined>(undefined);
  const [path, setPath] = useState<string>("");
  const [manager, setManager] = useState(exerciseContext.manager);
  let lastPose: number[] | undefined = undefined;
  const canvasRef = useRef<HTMLImageElement>(null);
  const vacuumSize = 40;
  let trail: number[][] = [];

  // Estado y referencias del audio
  const [status, setStatus] = useState<string>("Desconectado");
  const audioCtxRef = useRef<AudioContext | null>(null);
  const playerNodeRef = useRef<AudioWorkletNode | null>(null);

  const micStreamRef = useRef<MediaStream | null>(null);
  const micContextRef = useRef<AudioContext | null>(null);
  const micSourceRef = useRef<MediaStreamAudioSourceNode | null>(null);
  const micWorkletRef = useRef<AudioWorkletNode | null>(null);
  const micStartingRef = useRef(false);

  const [text, setText] = useState<string[]>([]);
  const [micStatus, setMicStatus] = useState<string>("Desconectado");
  const [isAudioReady, setAudioReady] = useState(false);
  const microphonePaused = useRef(false);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (!manager) {
      setAudioReady(false);
      return;
    }

    const state = manager.getState();
    const ready =
      state === states.TOOLS_READY ||
      state === states.RUNNING ||
      state === states.PAUSED;

    setAudioReady(ready);
    microphonePaused.current = state === states.PAUSED;
  }, [manager]);

  // Ya no es necesario clicar el botón para activar la conexión
  useEffect(() => {
    connectAudio();
  }, []);

  // Al desmontar el componente, cierra el contexto de audio
  useEffect(() => {
    return () => {
      audioCtxRef.current?.close();
      audioCtxRef.current = null;
      playerNodeRef.current = null;
    };
  }, []);

  const resizeObserver = new ResizeObserver((entries) => {
    const img = entries[0].target;
    const width = img.clientWidth / 300;
    const height = img.clientHeight / 150;

    updatePath(
      trail,
      setPath,
      height,
      (vacuumSize * img.clientHeight) / 1012,
      width,
      (vacuumSize * img.clientWidth) / 1012
    );

    if (lastPose) {
      setVacuumPose([
        lastPose[1] * height - (vacuumSize * img.clientHeight) / 1012,
        lastPose[0] * width - (vacuumSize * img.clientWidth) / 1012,
        -lastPose[2],
      ]);
    }
  });

  // Función que solicita permisos del micrófono y envía los fragmentos a la aplicación
  const startMicrophone = async () => {
    if (!manager || micContextRef.current || micStartingRef.current) return;

    micStartingRef.current = true;
    try {
      micStreamRef.current = await navigator.mediaDevices.getUserMedia({
        audio: true,
      });
      micContextRef.current = new AudioContext({ sampleRate: MICROPHONE_SAMPLE_RATE });

      const urlCode = getWorkletUrl();
      await micContextRef.current.audioWorklet.addModule(urlCode);
      URL.revokeObjectURL(urlCode);

      micSourceRef.current = micContextRef.current.createMediaStreamSource(
        micStreamRef.current,
      );
      // AudioWorletNode - audio crudo
      micWorkletRef.current = new AudioWorkletNode(
        micContextRef.current,
        "audio-worklet",
      );
      micSourceRef.current.connect(micWorkletRef.current);
      micWorkletRef.current.port.onmessage = (ev) => {
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

      setMicStatus("Escuchando");
    } catch (e) {
      setMicStatus(`Error de micrófono: ${(e as Error).message}`);
    } finally {
      micStartingRef.current = false;
    }
  };

  const stopMicrophone = async () => {
    if (micStreamRef.current && manager) {
      manager.send("gui", "audio-stop");
    }

    micWorkletRef.current?.disconnect();
    micSourceRef.current?.disconnect();

    micStreamRef.current?.getTracks().forEach((track) => track.stop());

    if (micContextRef.current) {
      await micContextRef.current.close();
    }

    micWorkletRef.current = null;
    micSourceRef.current = null;
    micStreamRef.current = null;
    micContextRef.current = null;

    setMicStatus("Desconectado");
  };

  useEffect(() => {
    if (isAudioReady && manager) {
      startMicrophone();
    } else {
      stopMicrophone();
    }

    return () => {
      stopMicrophone();
    };
  }, [isAudioReady, manager]);

  // Crea el AudioContext y el worklet (solo tras un clic del usuario). Devuelve false si falla
  const initAudio = async (): Promise<boolean> => {
    try {
      if (!audioCtxRef.current) {
        const AudioCtx =
          window.AudioContext ||
          (window as unknown as { webkitAudioContext: typeof AudioContext })
            .webkitAudioContext;
        const audioCtx = new AudioCtx({ sampleRate: SAMPLE_RATE });

        const blob = new Blob([WORKLET_CODE], {
          type: "application/javascript",
        });
        const workletUrl = URL.createObjectURL(blob);
        await audioCtx.audioWorklet.addModule(workletUrl);
        URL.revokeObjectURL(workletUrl);

        const player = new AudioWorkletNode(audioCtx, "pcm-player", {
          numberOfInputs: 0,
          numberOfOutputs: 1,
          outputChannelCount: [1],
        });
        player.connect(audioCtx.destination);

        audioCtxRef.current = audioCtx;
        playerNodeRef.current = player;
      }

      if (audioCtxRef.current.state === "suspended") {
        await audioCtxRef.current.resume();
      }
      return true;
    } catch (err) {
      console.error("Error inicializando Web Audio:", err);
      setStatus(`Error audio: ${(err as Error).message}`);
      return false;
    }
  };

  // Decodifica un trozo de PCM (base64, int16) y se lo manda al worklet
  const playChunk = (audioBase64: string) => {
    const player = playerNodeRef.current;
    if (!player) return;
    try {
      const binary = atob(audioBase64);
      const bytes = new Uint8Array(binary.length);
      for (let i = 0; i < binary.length; i++) {
        bytes[i] = binary.charCodeAt(i);
      }
      const int16 = new Int16Array(bytes.buffer);
      const float32 = Float32Array.from(int16, (v) => v / 32768);
      player.port.postMessage(float32, [float32.buffer]);
    } catch (e) {
      console.error("Error decodificando el audio PCM:", e);
    }
  };

  // update.audio es un JSON {"audio": base64, "status": str}
  const handleAudioUpdate = (rawAudio: string) => {
    let payload: { audio?: string; status?: string };
    try {
      payload = JSON.parse(rawAudio);
    } catch (e) {
      console.error("Error leyendo el mensaje de audio:", e);
      return;
    }

    if (payload.status) setStatus(statusLabel(payload.status));
    if (payload.audio) playChunk(payload.audio);
  };

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    if (update.text) {
      const texts = JSON.parse(update.text);
      if (texts.length > 0) {
        setText((prev) => [...prev, ...texts]);
      }
    }

    // Lógica para manejar el audio (va antes que el mapa porque este puede hacer return)
    if (update.audio) {
      handleAudioUpdate(update.audio);
    }

    // Lógica para manejar el mapa
    if (update.map) {
      const pose = update.map.substring(1, update.map.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));
      lastPose = content;

      const img = canvasRef.current;
      if (img === null) {
        return;
      }
      const width = img.clientWidth / 300;
      const height = img.clientHeight / 150;

      updatePath(
        trail,
        setPath,
        height,
        (vacuumSize * img.clientHeight) / 1012,
        width,
        (vacuumSize * img.clientWidth) / 1012
      );

      setVacuumPose([
        content[1] * height - (vacuumSize * img.clientHeight) / 1012,
        content[0] * width - (vacuumSize * img.clientWidth) / 1012,
        -content[2],
      ]);
      addToPath(content[1], content[0], trail);
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY || state === states.RUNNING) {
      setAudioReady(true);
      microphonePaused.current = false;
    } else if (state === states.PAUSED) {
      setAudioReady(true);
      microphonePaused.current = true;
    } else {
      setAudioReady(false);
    }

    if (state === states.TOOLS_READY) {
      setText([]);
    }
    if (state === states.TOOLS_READY) {
      setPath("");
      trail = [];
      setVacuumPose(undefined);
      setStatus("Listo");
    }
  };

  connectApplication(
    manager,
    updateCallback,
    stateCallback,
    canvasRef,
    resizeObserver
  );

  // Habilita el audio y le pide al backend que sintetice y mande el texto
  const connectAudio = async () => {
    if (!(await initAudio())) return;

    if (!manager) {
      // Sin Manager no hay a quién mandarle nada: avisamos en vez de fallar en silencio
      setStatus("Sin conexión con el Manager: pulsa «Click to connect» arriba");
      return;
    }

    playerNodeRef.current?.port.postMessage("clear");
    setStatus("Conectando...");
    manager.send("gui", JSON.stringify({ play: true }));
  };

  return (
    <WebGUIContainer>
      <WebGUIImage
        reference={canvasRef}
        id="map-img"
        src={houseMapDirty}
        style={{ left: "0", width: "100%" }}
      />
      <div className="overlay" id="map-container">
        {vacuumPose && canvasRef.current && (
          <div
            id="vacuum-pos"
            style={{
              rotate: "z " + vacuumPose[2] + "rad",
              top: vacuumPose[0],
              height: (vacuumSize * canvasRef.current.clientHeight) / 1012,
              left: vacuumPose[1],
              width: (vacuumSize * canvasRef.current.clientWidth) / 1012,
            }}
          >
            <img src={Vacuum} />
            <div
              className="arrow"
              style={{
                height: (vacuumSize * canvasRef.current.clientHeight) / 1012,
                marginLeft:
                  -((vacuumSize * canvasRef.current.clientWidth) / 1012) / 2,
              }}
            />
          </div>
        )}
        {/* <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg">
          <mask id="svg-draw" xmlns="http://www.w3.org/2000/svg">
            {path ? (
              <path xmlns="http://www.w3.org/2000/svg" d={path} style={{strokeWidth: "30px", strokeLinejoin:"round", stroke: "white", fill: "none"}}/>
            ) : (
              <path xmlns="http://www.w3.org/2000/svg"></path>
            )}
          </mask>
          <image href={houseMapClean} height="100%" width="100%" mask="url(#svg-draw)" preserveAspectRatio="none"></image>
        </svg> */}
        {path && (
          <svg
            height="100%"
            width="100%"
            xmlns="http://www.w3.org/2000/svg"
            style={{ zIndex: 2, position: "absolute", left: 0 }}
          >
            <path
              xmlns="http://www.w3.org/2000/svg"
              d={path}
              style={{
                strokeWidth:
                  (vacuumSize * canvasRef.current!.clientHeight) / 1012,
                strokeLinejoin: "round",
                stroke: "white",
                fill: "none",
              }}
            />
          </svg>
        )}
      </div>
    </WebGUIContainer>
  );
};

export default WebGUI;

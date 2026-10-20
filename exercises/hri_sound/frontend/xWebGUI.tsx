import { useState, useEffect, useRef } from "react";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";

// Código del AudioWorklet, cargado desde un Blob para que el archivo sea autocontenido.
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

const SAMPLE_RATE = 48000; // debe coincidir con SAMPLE_RATE de audio_source.py

// Estados que manda el backend -> texto que se muestra
const STATUS_LABELS: Record<string, string> = {
  idle: "Listo",
  synthesizing: "Sintetizando...",
  reading_file: "Leyendo archivo...",
  playing: "Reproduciendo",
  done: "Terminado",
  stopped: "Detenido",
};

type AudioUpdate = { update?: { audio?: string } };

// Traduce el estado del backend; si no lo conoce (p. ej. "error: ..."), lo deja tal cual
const statusLabel = (s: string) => STATUS_LABELS[s] ?? s;

function WebGUI() {
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);
  const [status, setStatus] = useState<string>("Desconectado");

  const audioCtxRef = useRef<AudioContext | null>(null);
  const playerNodeRef = useRef<AudioWorkletNode | null>(null);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  // Al desmontar el componente, cierra el contexto de audio
  useEffect(() => {
    return () => {
      audioCtxRef.current?.close();
      audioCtxRef.current = null;
      playerNodeRef.current = null;
    };
  }, []);

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

  // Mensajes del backend: update.audio es un JSON {"audio": base64, "status": str}
  const updateCallback = (updateData: unknown) => {
    const update = (updateData as AudioUpdate)?.update;
    if (!update?.audio) return;

    let payload: { audio?: string; status?: string };
    try {
      payload = JSON.parse(update.audio);
    } catch (e) {
      console.error("Error leyendo el mensaje de audio:", e);
      return;
    }

    if (payload.status) setStatus(statusLabel(payload.status));
    if (payload.audio) playChunk(payload.audio);
  };

  // Cambios de estado de la conexión con el Manager
  const stateCallback = (state: string) => {
    console.log("[audio_out] estado del Manager:", state);
    if (state === states.TOOLS_READY) {
      setStatus("Listo");
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  // Habilita el audio y le pide al backend que sintetice y mande el texto
  const connect = async () => {
    console.log("[audio_out] clic en conectar; manager =", manager);
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
      <div style={{ display: "flex", flexDirection: "column", gap: "8px" }}>
        <button onClick={connect}>Conectar y recibir audio</button>
        <p>{status}</p>
      </div>
    </WebGUIContainer>
  );
}

export default WebGUI;
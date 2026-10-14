import React, { useRef, useState } from "react";
import WebGUIContainer from "Components/exercise/WebGUIContainer";

// Código del worklet como texto: se carga desde un Blob para no depender de cómo sirva archivos estáticos el empaquetador del frontend
const WORKLET_CODE = `
  class PCMPlayerProcessor extends AudioWorkletProcessor {
    constructor() {
      super();
      this.queue = []; // cola de trozos (Float32Array) pendientes de reproducir
      this.offset = 0; // por dónde vamos dentro del primer trozo de la cola

      // El hilo principal nos manda trozos de audio por este puerto.
      this.port.onmessage = (event) => {
        this.queue.push(event.data);
      };
    }

    // El navegador llama a esto muy a menudo (cada ~128 muestras) para pedir el siguiente bloque de audio a reproducir
    process(_inputs, outputs) {
      const output = outputs[0][0];
      let written = 0;

      while (written < output.length) {
        if (this.queue.length === 0) {
          // No ha llegado audio a tiempo: rellenamos con silencio en vez de cortar.
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

      return true; // seguir vivo para el siguiente bloque
    }
  }

  registerProcessor('pcm-player', PCMPlayerProcessor);
`;

const SAMPLE_RATE = 48000; // debe coincidir con audio_source.py

function WebGUI() {
  const [status, setStatus] = useState("Desconectado");
  const audioCtxRef = useRef<AudioContext | null>(null);
  const socketRef = useRef<WebSocket | null>(null);

  const connect = async () => {
    setStatus("Conectando...");
    try {
      // El AudioContext se crea tras el clic (gesto del usuario)
      const audioCtx = new AudioContext({ sampleRate: SAMPLE_RATE });
      const blob = new Blob([WORKLET_CODE], { type: "application/javascript" });
      await audioCtx.audioWorklet.addModule(URL.createObjectURL(blob));

      const player = new AudioWorkletNode(audioCtx, "pcm-player", {
        numberOfInputs: 0,
        numberOfOutputs: 1,
        outputChannelCount: [1],
      });
      player.connect(audioCtx.destination);
      audioCtxRef.current = audioCtx;

    console.log("Abriendo WebSocket, página en:", window.location.href);
      const socket = new WebSocket("ws://127.0.0.1:8001/ws");
      socket.binaryType = "arraybuffer";
      socketRef.current = socket;

      socket.onmessage = (event) => {
        // Texto = JSON de estado; binario = frame PCM de 16 bits
        if (typeof event.data === "string") {
          const msg = JSON.parse(event.data);
          if (msg.status) setStatus(`Estado: ${msg.status}`);
          if (msg.error) setStatus(`Error: ${msg.error}`);
          return;
        }
        const int16 = new Int16Array(event.data);
        const float32 = Float32Array.from(int16, (v) => v / 32768);
        player.port.postMessage(float32, [float32.buffer]);
      };
      //socket.onclose = () => setStatus("Desconectado.");
      //
      socket.onopen = () => {
      console.log("WS abierto");
      setStatus("Conectado. Esperando audio...");
    };

    socket.onerror = (e) => {
      console.error("WS error", e);
      setStatus("No se pudo conectar con el servidor.");
    };

    socket.onclose = (e) => {
      console.log("WS close", {
        code: e.code,
        reason: e.reason,
        wasClean: e.wasClean,
        url: socket.url,
      });
      // No pisar un mensaje de error que ya se haya mostrado
      setStatus((prev) =>
        prev.startsWith("No se pudo") || prev.startsWith("Error")
          ? prev
          : "Desconectado."
      );
    };
    //
    } catch (err) {
      setStatus(`Error: ${(err as Error).message}`);
    }
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
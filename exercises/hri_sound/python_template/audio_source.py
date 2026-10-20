# Fuente de audio: lee un archivo y lo entrega como PCM en frames de 20 ms.
#
# Contrato: PCM de 16 bits, mono, 48 kHz, en bloques de exactamente 20 ms
# (960 muestras = 1920 bytes). Es el formato que espera el AudioWorklet del
# navegador, así que ni el servidor ni el cliente tienen que hacer más
# conversiones que las de aquí.
#
# La fuente NO controla el ritmo: entrega frames tan rápido como se le piden.
# Quien consume (el bucle de envío por WebSocket, en server.py) decide cada
# cuánto pedir el siguiente frame.

import asyncio
from pathlib import Path
from typing import AsyncIterator

import av

SAMPLE_RATE = 48_000
FRAME_MS = 20
SAMPLES_PER_FRAME = SAMPLE_RATE * FRAME_MS // 1000  # 960
BYTES_PER_SAMPLE = 2  # PCM s16
FRAME_BYTES = SAMPLES_PER_FRAME * BYTES_PER_SAMPLE  # 1920


class FrameChunker:
    # Trocea PCM de tamaño irregular en frames de 20 ms exactos.

    def __init__(self) -> None:
        self._buffer = bytearray()

    def push(self, pcm: bytes) -> list[bytes]:
        # Añade PCM y devuelve todos los frames completos que ya se puedan sacar.
        self._buffer.extend(pcm)
        frames = []
        while len(self._buffer) >= FRAME_BYTES:
            frames.append(bytes(self._buffer[:FRAME_BYTES]))
            del self._buffer[:FRAME_BYTES]
        return frames

    def flush(self) -> bytes | None:
        # Devuelve el resto pendiente, relleno de silencio hasta completar 20 ms.
        if not self._buffer:
            return None
        frame = bytes(self._buffer).ljust(FRAME_BYTES, b"\x00")
        self._buffer.clear()
        return frame


async def file_frames(path: str | Path) -> AsyncIterator[bytes]:
    # Decodifica un archivo de audio y lo entrega como frames PCM de 20 ms.
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"No se encuentra el archivo: {path}")

    chunker = FrameChunker()
    resampler = av.AudioResampler(format="s16", layout="mono", rate=SAMPLE_RATE)

    with av.open(str(path)) as container:
        stream = container.streams.audio[0]
        for decoded in container.decode(stream):
            for resampled in resampler.resample(decoded):
                for frame in chunker.push(resampled.to_ndarray().tobytes()):
                    yield frame
            await asyncio.sleep(0)  # cede el control entre frames decodificados

        for resampled in resampler.resample(None):  # vacía el remuestreador
            for frame in chunker.push(resampled.to_ndarray().tobytes()):
                yield frame

    last = chunker.flush()
    if last is not None:
        yield last
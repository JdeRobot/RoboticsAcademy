# Fuente de audio: sintetiza texto a voz con Piper y lo entrega como PCM
# en frames de 20 ms, igual que audio_source.py pero a partir de texto en
# vez de un archivo.

import asyncio
from typing import AsyncIterator

import av
import numpy as np
from piper import PiperVoice

from audio_source import SAMPLE_RATE, FrameChunker


async def text_frames(text: str, voice: PiperVoice) -> AsyncIterator[bytes]:
    # Sintetiza `text` con `voice` y entrega frames PCM de 20 ms a 48 kHz.
    #
    # Piper sintetiza frase a frase (un AudioChunk por frase), así que cada
    # frase se remuestrea y trocea en cuanto está lista, sin esperar a que
    # termine de sintetizarse el texto completo.
    chunker = FrameChunker()
    resampler = av.AudioResampler(format="s16", layout="mono", rate=SAMPLE_RATE)

    synth = voice.synthesize(text)  # generador síncrono de Piper
    sentinel = object()

    while True:
        # next() ejecuta el modelo y bloquea; lo corremos en un hilo aparte
        # para no congelar el servidor (y sus otras conexiones) mientras
        # Piper sintetiza la siguiente frase.
        chunk = await asyncio.to_thread(next, synth, sentinel)
        if chunk is sentinel:
            break

        samples = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16).reshape(1, -1)
        frame = av.AudioFrame.from_ndarray(samples, format="s16", layout="mono")
        frame.sample_rate = chunk.sample_rate  # Piper no siempre usa 48 kHz

        for resampled in resampler.resample(frame):
            for pcm_frame in chunker.push(resampled.to_ndarray().tobytes()):
                yield pcm_frame

    for resampled in resampler.resample(None):  # vacía el remuestreador
        for pcm_frame in chunker.push(resampled.to_ndarray().tobytes()):
            yield pcm_frame

    last = chunker.flush()
    if last is not None:
        yield last
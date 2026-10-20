import asyncio
import time
import wave
from pathlib import Path

from aiohttp import web, WSMsgType
from piper import PiperVoice

from audio_source import FRAME_MS, SAMPLE_RATE, SAMPLES_PER_FRAME, file_frames
from piper_source import text_frames

PROJECT_DIR = Path(__file__).parent
# Archivos que no vienen incluidos en los requerimientos del proyecto
PIPER_MODEL_PATH = PROJECT_DIR / "es_ES-carlfm-x_low.onnx"
PIPER_CONFIG_PATH = PROJECT_DIR / "es_ES-carlfm-x_low.onnx.json" # posible necesidad de cambiar la ruta a absoluta

voice: PiperVoice | None = None

async def index(request: web.Request) -> web.Response:
    # Sirve la página HTML tal cual
    html = (PROJECT_DIR / "index.html").read_text()
    return web.Response(content_type="text/html", text=html)


async def worklet(request: web.Request) -> web.Response:
    # Sirve el AudioWorklet como JavaScript
    js = (PROJECT_DIR / "pcm-player-worklet.js").read_text()
    return web.Response(content_type="application/javascript", text=js)


async def send_paced(ws: web.WebSocketResponse, frames) -> None:
    # Manda los frames por el WebSocket, uno cada 20 ms reales
    start_time = None  # instante real en que empieza el envío
    samples_sent = 0  # muestras mandadas hasta ahora, para calcular el pacing

    async for frame in frames:
        if ws.closed:
            return

        await ws.send_bytes(frame)
        samples_sent += SAMPLES_PER_FRAME

        if start_time is None:
            start_time = time.time()
            continue

        # Sin esto, como decodificar es más rápido que tiempo real, se
        # mandaría todo el audio de golpe en vez de a ritmo de reproducción
        target = start_time + samples_sent / SAMPLE_RATE
        delay = target - time.time()
        if delay > 0:
            await asyncio.sleep(delay)


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    # Cada conexión WebSocket recibe el audio del archivo en streaming
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    text = "En un lugar de la Mancha, de cuyo nombre no quiero acordarme."

    await ws.send_json({"status": "synthesizing"})
    await send_paced(ws, text_frames(text, voice))
    await ws.send_json({"status": "done"})
    await ws.close()

    return ws


def build_app() -> web.Application:
    # Crea la aplicación aiohttp con sus tres rutas, y carga la voz de Piper
    global voice
    voice = PiperVoice.load(str(PIPER_MODEL_PATH), config_path=str(PIPER_CONFIG_PATH))

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/pcm-player-worklet.js", worklet)
    app.router.add_get("/ws", websocket_handler)
    return app


if __name__ == "__main__":
    web.run_app(build_app(), host="0.0.0.0", port=8001)

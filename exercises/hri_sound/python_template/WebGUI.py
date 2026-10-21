import json
import threading
import rclpy
import asyncio
import base64
import json
import sys
import threading
from collections import deque
from pathlib import Path
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from gui_interfaces.general.measuring_threading_gui_no_sim import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from map import Map

# Simple HAL check
try:
    from HAL import getPose3d as hal_getPose3d

    HAL_AVAILABLE = True
except ImportError:
    HAL_AVAILABLE = False


# Carpeta donde se ejecuta este archivo (el Manager copia aquí python_template, con audio_source.py y piper_source.py)
try:
    PROJECT_DIR = Path(__file__).resolve().parent
except NameError:
    PROJECT_DIR = Path.cwd()

# Ruta del ejercicio dentro del contenedor (el repo está montado en /RoboticsAcademy)
EXERCISE_DIR = Path("/RoboticsAcademy/exercises/hri_sound")

# Garantiza que podamos importar audio_source.py y piper_source.py, aunque WebGUI.py se ejecute desde otra carpeta
sys.path.insert(0, str(PROJECT_DIR))
sys.path.append(str(EXERCISE_DIR / "python_template"))

# Estos imports van después de ajustar sys.path
from audio_source import file_frames  # noqa: E402
from piper_source import text_frames  # noqa: E402
from piper import PiperVoice  # noqa: E402


MODEL_NAME = "es_ES-carlfm-x_low.onnx"
MODEL_DIRS = [EXERCISE_DIR / "models", PROJECT_DIR / "models", PROJECT_DIR]

DEFAULT_TEXT = "Con cien cañones por banda"

# Máximo de frames de 20 ms que se mandan al navegador en cada ciclo de update_gui (25 frames = 0,5 s de audio). Si el ciclo es lento, igualmente llega audio de sobra; el navegador lo encola y lo reproduce a su ritmo.
MAX_FRAMES_PER_CYCLE = 25

# Estados en los que hay algo sintetizándose o sonando
BUSY_STATUSES = ("synthesizing", "reading_file", "playing")


class ROS2Node(Node):
    def __init__(self):
        super().__init__("gui_data")
        self.pose = None
        self.pose_lock = threading.Lock()  # Thread safety
        self.create_subscription(Odometry, "/vacuum_cleaner/odom", self.callback, 10)

    def callback(self, msg):
        with self.pose_lock:
            self.pose = msg.pose.pose

    def get_pose3d(self):
        with self.pose_lock:
            if not self.pose:

                class P:
                    x = y = yaw = 0.0

                return P()

            import math

            q = self.pose.orientation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )

            class Pose3D:
                def __init__(self, x, y, yaw):
                    self.x, self.y, self.yaw = x, y, yaw

            return Pose3D(self.pose.position.x, self.pose.position.y, yaw)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars (map: ejercicio vacuum / audio: ejercicio audio_out)
        self.payload = {"map": "", "audio": ""}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        # Initialize ROS2 components
        self.ros_node = None
        self.executor = None
        self.executor_thread = None

        # --- Audio (audio_out) ---
        # Estado del audio. Se usa "audio_status" (y no "status") para no pisar ningún atributo que ya tenga la clase base.
        self.audio_status = "idle"
        self.audio_lock = threading.Lock()

        # Condition sobre el mismo lock: despierta al hilo de síntesis cuando llega un pedido
        self.speech_cond = threading.Condition(self.audio_lock)
        self.audio_queue = deque()  # frames PCM de 20 ms pendientes de enviar al navegador
        self.speech_queue = deque()  # pedidos pendientes: ("text", texto) o ("file", ruta)
        self.busy = False  # True mientras el hilo de síntesis procesa un pedido
        self.generation = 0  # se incrementa en cada interrupción para cancelar el pedido en curso
        self.synthesis_done = False  # True cuando el worker ha terminado de encolar frames

        # Modelo de voz de Piper
        model_dir = self._find_model_dir()
        self.piper_model_path = model_dir / MODEL_NAME
        self.piper_config_path = model_dir / (MODEL_NAME + ".json")
        self.voice = None
        self._load_voice()

        # Auto-detect mode and get pose function
        pose_getter = self._get_pose_function()
        self.map = Map(pose_getter)
        self.start()

    def _get_pose_function(self):
        """Auto-detect the best pose source"""
        # Try ROS2 first
        try:
            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()

            # Check for topics using a temporary node
            temp_node = rclpy.create_node("temp_topic_checker")
            topics = [name for name, _ in temp_node.get_topic_names_and_types()]
            temp_node.destroy_node()

            if "/odom" in topics:
                # Set up ROS2 node with proper executor
                self.ros_node = ROS2Node()
                self.executor = SingleThreadedExecutor()
                self.executor.add_node(self.ros_node)

                # Start executor in separate thread
                self.executor_thread = threading.Thread(
                    target=self._ros_spin, daemon=True
                )
                self.executor_thread.start()

                print("GUI: ROS2 mode activated - listening to /odom")
                return self.ros_node.get_pose3d

        except Exception as e:
            print(f"GUI: ROS2 setup failed: {e}")

        # Fallback to HAL
        if HAL_AVAILABLE:
            print("GUI: HAL mode activated")
            return hal_getPose3d

        # Last resort: dummy mode
        print("GUI: Dummy mode activated - no pose data available")

        class DummyPose:
            x = y = yaw = 0.0

        return lambda: DummyPose()

    def _ros_spin(self):
        """Safe ROS2 spinning in dedicated thread"""
        try:
            self.executor.spin()
        except Exception as e:
            print(f"GUI: ROS2 executor error: {e}")

    def update_gui(self):
        """Update GUI with latest robot position and pending audio"""
        pos = self.map.getRobotCoordinates()
        if pos == self.init_coords:
            pos = self.start_coords
        ang = self.map.getRobotAngle()
        self.payload["map"] = str(pos + ang)
        self.payload["audio"] = json.dumps(self.payload_audio())
        self.send_to_client(json.dumps(self.payload))

    def reset_gui(self):
        """Reset the map"""
        self.map.reset()

    def __del__(self):
        """Cleanup ROS2 resources"""
        if self.executor:
            self.executor.shutdown()

    # ------------------------------------------------------------------
    # Audio (audio_out)
    # ------------------------------------------------------------------

    # Devuelve la primera carpeta de MODEL_DIRS que contiene el modelo de Piper
    def _find_model_dir(self):
        for model_dir in MODEL_DIRS:
            if (model_dir / MODEL_NAME).exists():
                return model_dir
        print(f"No se encuentra {MODEL_NAME} en {[str(d) for d in MODEL_DIRS]}", file=sys.stderr)
        return MODEL_DIRS[0]

    # Carga el modelo de Piper; deja self.voice en None si falla
    def _load_voice(self):
        try:
            self.voice = PiperVoice.load(
                str(self.piper_model_path), config_path=str(self.piper_config_path)
            )
            print("Modelo de voz de Piper cargado")
        except Exception as e:
            print(f"Error cargando el modelo de Piper: {e}", file=sys.stderr)
            self.voice = None

    def gui_in_thread(self, ws, message):
        # Mensajes que llegan del frontend (manager.send("gui", ...))

        # Orden de reproducir: JSON {"play": true, "text": "..."}.
        # Se procesa primero y se sale, para que el texto del usuario no se
        # confunda con las palabras clave que gestiona la clase base (p. ej. un texto con "stop").
        if message.startswith("{"):
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                return
            if data.get("play"):
                self.play_text(data.get("text") or DEFAULT_TEXT)
            return

        # Resto de mensajes (ack, start, introspection...): los gestiona la clase base
        super().gui_in_thread(ws, message)

    def payload_audio(self):
        # Saca de la cola hasta MAX_FRAMES_PER_CYCLE frames y los pasa a base64
        with self.audio_lock:
            chunks = []
            while self.audio_queue and len(chunks) < MAX_FRAMES_PER_CYCLE:
                chunks.append(self.audio_queue.popleft())

            # Cola vacía y el worker ha terminado: la reproducción ha acabado
            if (
                not self.audio_queue
                and self.synthesis_done
                and self.audio_status in ("synthesizing", "playing")
            ):
                self.audio_status = "done"
                self.synthesis_done = False

            status = self.audio_status

        audio_b64 = base64.b64encode(b"".join(chunks)).decode("utf-8") if chunks else ""
        return {"audio": audio_b64, "status": status}

    def play_text(self, text=DEFAULT_TEXT):
        # Sintetiza el texto con Piper en un hilo aparte y lo va encolando
        gen = self._new_generation("synthesizing")
        threading.Thread(
            target=self._synthesize_and_queue,
            args=(text, gen),
            daemon=True,
            name="audio_synthesis_worker",
        ).start()

    def play_file(self, file_path):
        # Lee un archivo de audio en un hilo aparte y lo va encolando
        gen = self._new_generation("reading_file")
        threading.Thread(
            target=self._read_and_queue_file,
            args=(file_path, gen),
            daemon=True,
            name="audio_file_reader_worker",
        ).start()

    def _new_generation(self, first_status):
        # Cancela cualquier worker anterior, vacía la cola y devuelve el id del nuevo
        with self.audio_lock:
            self.generation += 1
            self.audio_queue.clear()
            self.synthesis_done = False
            self.audio_status = first_status
            return self.generation

    # Worker de TTS: Piper -> frames de 20 ms -> cola
    def _synthesize_and_queue(self, text, gen):
        if self.voice is None:
            self._load_voice()
        if self.voice is None:
            self._fail(gen, "voice model not loaded")
            return
        self._run_worker(text_frames(text, self.voice), gen, "synthesizing")

    # Worker de archivo: decodifica el archivo -> frames de 20 ms -> cola
    def _read_and_queue_file(self, file_path, gen):
        self._run_worker(file_frames(file_path), gen, "reading_file")

    # Consume los frames y los encola mientras este worker siga siendo el vigente
    def _run_worker(self, frames, gen, waiting_status):
        async def _run():
            async for frame in frames:
                with self.audio_lock:
                    if gen != self.generation:
                        return  # cancelado por un play/stop posterior
                    self.audio_queue.append(frame)
                    if self.audio_status == waiting_status:
                        self.audio_status = "playing"

        try:
            asyncio.run(_run())
            with self.audio_lock:
                if gen == self.generation:
                    self.synthesis_done = True
        except Exception as e:
            print(f"Error en el worker de audio: {e}", file=sys.stderr)
            self._fail(gen, str(e))

    # Marca error en el estado, solo si el worker sigue siendo el vigente
    def _fail(self, gen, reason):
        with self.audio_lock:
            if gen == self.generation:
                self.audio_status = f"error: {reason}"

    # Corta la reproducción: cancela workers y vacía la cola
    def stop(self):
        with self.audio_lock:
            self.generation += 1
            self.audio_queue.clear()
            self.synthesis_done = False
            self.audio_status = "stopped"

    # True si hay audio sintetizándose o pendiente de enviar
    def is_playing(self):
        with self.audio_lock:
            return self.audio_status in BUSY_STATUSES or len(self.audio_queue) > 0

    # Estado actual del audio
    def get_status(self):
        with self.audio_lock:
            return self.audio_status


# Create GUI instance
gui = WebGUI()
start_console()


# FUNCIONES API PARA EL USUARIO (audio_out)

# Reproduce un texto
def play(text=DEFAULT_TEXT):
    gui.play_text(text)


# Alias de play()
def say(text):
    gui.play_text(text)


# Reproduce un archivo de audio
def play_file(file_path):
    gui.play_file(file_path)


# Detiene la reproducción
def stop():
    gui.stop()


# Estado actual
def get_status():
    return gui.get_status()


# ¿Está sonando algo?
def is_playing():
    return gui.is_playing()
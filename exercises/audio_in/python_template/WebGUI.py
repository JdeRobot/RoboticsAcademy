import json

# import cv2
import base64
import threading

# import sys
import torch
from faster_whisper import WhisperModel

from gui_interfaces.general.measuring_threading_gui_no_sim import MeasuringThreadingGUI
from console_interfaces.general.console import start_console

import numpy as np


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # GUI display state
        self.display_text = []
        self.display_text_lock = threading.Lock()
        self.transcriptions = []
        
        self.last_timestamp = 0

        self.payload = {"text": ""}

        self.sample_rate = 16000
        self.language = "es"
        self.model_size = "base"
        self.vad_threshold = 0.5
        self.grace_period = 0.5
        self.max_phrase_secs = 3.0

        self.chunk_size = 512
        self.max_silence = int(self.grace_period * self.sample_rate / self.chunk_size)
        self.max_phrase_chunks = int(
            self.max_phrase_secs * self.sample_rate / self.chunk_size
        )
        self.min_phrase_chunks = 5

        self.silero_model, _ = torch.hub.load(
            repo_or_dir="snakers4/silero-vad", model="silero_vad", trust_repo=True
        )
        self.whisper = WhisperModel(
            self.model_size, device="cpu", compute_type="int8", cpu_threads=4
        )

        self.phrase = []
        self.silence = 0

        self.start()

    def gui_in_thread(self, ws, message):
        time_frame_size = 20

        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        elif "start" in message:
            with self.ack_lock:
                self.ack_frontend = True

        if message.startswith("pick"):
            # Audio received from the frontend
            base64_buffer = message[4:-time_frame_size]
            self.last_timestamp = float(message[-time_frame_size:])

            audio_data = base64.b64decode(base64_buffer)
            audio_chunk = np.frombuffer(audio_data, dtype=np.float32)

            tensor_chunk = torch.from_numpy(audio_chunk.flatten()).float()
            speech_prob = self.silero_model(tensor_chunk, self.sample_rate).item()
            is_voice = speech_prob > self.vad_threshold

            if is_voice:
                self.phrase.append(audio_chunk)
                self.silence = 0
            elif self.phrase:
                # Silence after talking
                self.phrase.append(audio_chunk)
                self.silence += 1

            if self.silence > self.max_silence and self.phrase:
                self.transcribe_audio(self.last_timestamp)
                self.phrase = []
                self.silence = 0
            if len(self.phrase) > self.max_phrase_chunks:
                self.transcribe_audio(self.last_timestamp)
                self.phrase = []
                self.silence = 0

            # with self.latest_input_frame_lock:
            #     ack_message = {"ack_img": "ack", "time": time}
            #     self.send_to_client(json.dumps(ack_message))
        elif "audio-stop" in message:
            self.transcribe_audio(self.last_timestamp)
            self.phrase = []
            self.silence = 0

    def transcribe_audio(self, time: float) -> None:
        chunks = self.phrase

        if len(chunks) <= self.min_phrase_chunks:
            return

        audio = np.concatenate(chunks).flatten()
        segments, _ = self.whisper.transcribe(
            audio,
            language=self.language,
            beam_size=3,
            temperature=0.0,
            condition_on_previous_text=False,
        )

        text = "".join(segment.text for segment in segments).strip()

        if text:
            with self.display_text_lock:
                self.display_text.append(text)
            self.transcriptions.append({
              "text": text,
              "timestamp": time
            })

    def update_gui(self):
        payload = self.payloadText()
        self.payload["text"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def payloadText(self):
        with self.display_text_lock:
            payload = self.display_text
            self.display_text = []

        return payload
      
    def listen(self, ms: float):
      for transcription in self.transcriptions:
        if transcription["timestamp"] >= (self.last_timestamp - ms):
          print(transcription["text"])
    
    def printTexts(self):
      print(self.display_text)


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()

def listen(ms: float):
  gui.listen(ms)

def printTexts():
  gui.printTexts()
import asyncio
import serial
import time
from collections import deque
import random
import threading
from dataclasses import dataclass
import cv2
import numpy as np
import time
import os

# ---------------- CONFIG ----------------
PORT = "/dev/ttyUSB0"                # <<-- change to your Arduino port, e.g. "/dev/ttyUSB0" , "COM3" or "/dev/ttyUSB1"
BAUD = 9600

WAITING_TIME = 10            # seconds (for testing). Set to 10*60 for 10 minutes in production.
FEEDING_WAIT_AFTER_CMD = 30  # seconds fallback if Arduino doesn't respond (tunable)

TARGET_CLASS = "animal"
WINDOW_SECONDS = 3
ERASE_SECONDS = 7
DETECTION_THRESHOLD = 0.9
FIRST_IMAGE_MIN_AGE = 5.0    # "first valid image is old enough" requirement

IMAGE_SERVER_HOST = "0.0.0.0"
IMAGE_SERVER_PORT = 9000

# ---------------- STATES ----------------
WAITING = "WAITING"
DETECTING = "DETECTING"
FEEDING = "FEEDING"
EMPTY = "EMPTY"

# ---------------- classifier config ----------------

PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"
CONFIDENCE_THRESHOLD = 0.5
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

COLOR_BIRD = (0, 255, 0)   # Verde
COLOR_OTHER = (0, 0, 255)  # Vermelho


# ---------------- classifier ----------------

class VisionSystem:
    def __init__(self):
        print("[INFO] Loading model MobileNetSSD...")
        if not os.path.exists(PROTOTXT) or not os.path.exists(MODEL):
            raise FileNotFoundError("MobileNet model files not found!")
        self.net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)
        self.class_name = "bird"

        time.sleep(1.0) # Waiting for the camera to warm

    def classify_frame(self, frame):
        """
        Recebe a rede neural e um frame de vídeo.
        Processa a imagem, desenha os bounding boxes e retorna:
        - True: Se encontrou um pássaro.
        - False: Se não encontrou.
        Nota: O 'frame' é modificado in-place (desenha direto nele).
        """
        # 1. Prepara a imagem para a IA (Blob)
        h, w = frame.shape[:2]
        # Resize para 300x300 (padrão MobileNet) e normalização de cores
        resized_frame = cv2.resize(frame, (300, 300))
        blob = cv2.dnn.blobFromImage(resized_frame, 0.007843, (300, 300), 127.5)
        
        # 2. Executa a detecção
        self.net.setInput(blob)
        detections = self.net.forward()

        found_bird = False

        # 3. Analisa os resultados
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            
            if confidence > CONFIDENCE_THRESHOLD:
                idx = int(detections[0, 0, i, 1])
                class_name = CLASSES[idx]
                
                # Calcula coordenadas do quadrado
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Formata o texto da etiqueta
                label = "{}: {:.2f}%".format(class_name, confidence * 100)

                if class_name == self.class_name:
                    found_bird = True
                    # Desenha quadrado VERDE (Pássaro)
                    cv2.rectangle(frame, (startX, startY), (endX, endY), COLOR_BIRD, 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_BIRD, 2)
                else:
                    # Desenha quadrado VERMELHO (Outros objetos - opcional para debug)
                    cv2.rectangle(frame, (startX, startY), (endX, endY), COLOR_OTHER, 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_OTHER, 2)

        return found_bird

def mock_classify_image(img_bytes: bytes) -> str:
    # 20% chance "animal", 80% "other"
    return TARGET_CLASS if random.random() > 0 else "other"

# ---------------- state holder ----------------
class StateHolder:
    def __init__(self, initial_state: str):
        self._state = initial_state
        self._lock = asyncio.Lock()

    async def get(self) -> str:
        async with self._lock:
            return self._state

    async def set(self, new_state: str):
        async with self._lock:
            self._state = new_state

# ---------------- RollingDetector (async-safe) ----------------
class RollingDetector:
    def __init__(self):
        self.window = deque()  # deque[(timestamp, label)]
        self.lock = asyncio.Lock()

    async def push(self, label: bool):
        async with self.lock:
            print(f"adicionou {label}")
            now = time.time()
            self.window.append((now, label))

    async def flush(self):
        async with self.lock:
            self.window.clear()

    async def evaluate(self) -> bool:
        """
        Returns True only if:
         - after purging older-than ERASE_SECONDS,
         - the first (oldest) item in the remaining window is at least FIRST_IMAGE_MIN_AGE seconds old,
         - and >= DETECTION_THRESHOLD of the items within WINDOW_SECONDS are True.
        """
        async with self.lock:
            now = time.time()
            cutoff_erase = now - ERASE_SECONDS

            # purge too-old frames
            while self.window and self.window[0][0] < cutoff_erase:
                self.window.popleft()

            if not self.window:
                return False

            oldest_ts = self.window[0][0]
            if (now - oldest_ts) < FIRST_IMAGE_MIN_AGE:
                # ainda não temos imagens antigas o suficiente
                return False

            cutoff = now - WINDOW_SECONDS
            # queremos os items com timestamp >= cutoff (ou seja, mais recentes dentro da janela)
            recent_items = [(ts, lbl) for ts, lbl in self.window if ts >= cutoff]

            total = len(recent_items)
            if total == 0:
                return False

            positives = sum(1 for ts, lbl in recent_items if lbl == True)
            ratio = positives / total
            print(f"[detector] positives={positives} total={total} ratio={ratio:.2f}")
            return ratio >= DETECTION_THRESHOLD

    
    async def size(self) -> int:
        async with self.lock:
            return len(self.window)

# ---------------- Serial integration: background reader thread -> asyncio.Queue ----------------
def serial_reader_thread(ser: serial.Serial, queue: asyncio.Queue, stop_event: threading.Event, loop: asyncio.AbstractEventLoop):
    """
    Blocking reader running in a thread; lines are pushed into the asyncio queue.
    Each line is decoded and stripped; newline is the delimiter.
    """
    try:
        while not stop_event.is_set():
            try:
                line = ser.readline()  # blocking
                if not line:
                    continue
                try:
                    decoded = line.decode(errors="ignore").strip()
                except Exception:
                    decoded = line.decode("utf-8", errors="ignore").strip()
                # Use the loop passed in (safe)
                loop.call_soon_threadsafe(queue.put_nowait, decoded)
            except serial.SerialException:
                time.sleep(0.1)
    except Exception as e:
        print("[serial_thread] exception:", e)


async def async_serial_writer(ser: serial.Serial, msg: str):
    """
    Non-blocking write via executor (so we don't block event loop).
    """
    loop = asyncio.get_running_loop()
    def sync_write():
        ser.write((msg + "\n").encode())
        ser.flush()
    await loop.run_in_executor(None, sync_write)

# ---------------- Helper: wait for certain serial messages ----------------
async def wait_for_serial(queue: asyncio.Queue, expected_set: set, timeout: float = None):
    """
    Consume messages from queue until one is in expected_set (returns it).
    If timeout is provided and expires, raises asyncio.TimeoutError.
    Note: this function will discard messages that are not expected.
    """
    if timeout is None:
        while True:
            msg = await queue.get()
            if msg in expected_set:
                return msg
    else:
        deadline = asyncio.get_event_loop().time() + timeout
        while True:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                raise asyncio.TimeoutError()
            try:
                msg = await asyncio.wait_for(queue.get(), timeout=remaining)
            except asyncio.TimeoutError:
                raise
            if msg in expected_set:
                return msg

# ---------------- Async image server ----------------
async def handle_image_connection(reader: asyncio.StreamReader, writer: asyncio.StreamWriter,
                                  detector: RollingDetector, state_holder: StateHolder, vision: VisionSystem):
    peer = writer.get_extra_info("peername")
    print(f"[img_server] connection from {peer}")

    try:
        while True:
            # ---- RECEBE O FRAME ----
            header = await reader.readexactly(4)
            img_size = int.from_bytes(header, "big")
            img_bytes = await reader.readexactly(img_size)
            print("Received image")

            # ---- DECODIFICA PARA Numpy (necessário para desenhar e exibir) ----
            np_data = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

            if frame is None:
                print("[ERROR] Frame recebido não pode ser decodificado!")
                continue

            # ---- CLASSIFICA com desenhamento ----
            # vision.classify_frame modifica o frame desenhando bounding boxes
            label = vision.classify_frame(frame)

            # ---- MOSTRA AO VIVO (como no seu código exemplo) ----
            cv2.imshow("Deteccao em Tempo Real (Servidor)", frame)
            cv2.waitKey(1)

            # ---- EMPILHA PARA A JANELA DESLIZANTE ----
            if await state_holder.get() == DETECTING:
                await detector.push(label)

    except asyncio.IncompleteReadError:
        print(f"[img_server] connection closed by {peer}")

    except Exception as e:
        print("[img_server] error:", e)

    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass

        # Necessário evitar travamento da janela OpenCV quando o cliente desconecta
        cv2.destroyAllWindows()


async def start_image_server(detector: RollingDetector, state_holder: StateHolder, vision: VisionSystem):
    server = await asyncio.start_server(
        lambda r, w: handle_image_connection(r, w, detector, state_holder, vision),
        host=IMAGE_SERVER_HOST,
        port=IMAGE_SERVER_PORT
    )
    print(f"[img_server] listening on {IMAGE_SERVER_HOST}:{IMAGE_SERVER_PORT}")
    async with server:
        await server.serve_forever()

# ---------------- Main state machine ----------------
async def main():
    # setup serial
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"[main] Could not open serial port {PORT}: {e}")
        return

    # small pause to let Arduino reset if needed
    await asyncio.sleep(2.0)

    serial_queue = asyncio.Queue()   # lines from Arduino
    stop_event = threading.Event()
    loop = asyncio.get_running_loop()
    t = threading.Thread(target=serial_reader_thread, args=(ser, serial_queue, stop_event, loop), daemon=True)
    t.start()


    detector = RollingDetector()
    vision = VisionSystem()
    state_holder = StateHolder(WAITING)

    # start image server
    server_task = asyncio.create_task(start_image_server(detector, state_holder, vision))

    print("[main] starting state machine")
    try:
        while True:
            state = await state_holder.get()

            if state == WAITING:
                print("[main] state=WAITING -> sleeping for WAITING_TIME (async)")
                await asyncio.sleep(WAITING_TIME)
                await state_holder.set(DETECTING)
                print("[main] transitioned to DETECTING; flushing detector and notifying Arduino")
                await detector.flush()
                await async_serial_writer(ser, "STATE_DETECTING")

            elif state == DETECTING:
                is_detected = await detector.evaluate()
                if is_detected:
                    print("[main] DETECTION EVENT: flushing frames and sending STATE_FEEDING")
                    await detector.flush()
                    await state_holder.set(FEEDING)
                    await async_serial_writer(ser, "STATE_FEEDING")
                else:
                    print("not detected")
                    # not detected yet: just wait a small tick
                    await asyncio.sleep(0.1)

            elif state == FEEDING:
                try:
                    msg = await wait_for_serial(serial_queue, {"DONE_FEEDING", "EMPTY"}, timeout=3.0)
                except asyncio.TimeoutError:
                    msg = None
                    print("[main] No immediate DONE_FEEDING/EMPTY reply (3s timeout). Will wait longer as fallback.")

                if msg == "DONE_FEEDING":
                    print("[main] Received DONE_FEEDING -> transition to WAITING")
                    await state_holder.set(WAITING)
                    await async_serial_writer(ser, "STATE_WAITING")

                elif msg == "EMPTY":
                    print("[main] Received EMPTY -> transition to EMPTY")
                    await state_holder.set(EMPTY)

                else:
                    # No immediate reply: wait a longer time for either DONE_FEEDING or EMPTY
                    try:
                        msg2 = await wait_for_serial(serial_queue, {"DONE_FEEDING", "EMPTY"}, timeout=FEEDING_WAIT_AFTER_CMD)
                    except asyncio.TimeoutError:
                        msg2 = None
                        print(f"[main] No DONE_FEEDING/EMPTY after fallback timeout ({FEEDING_WAIT_AFTER_CMD}s). Falling back to WAITING.")

                    if msg2 == "DONE_FEEDING":
                        print("[main] Received DONE_FEEDING (fallback) -> WAITING")
                        await state_holder.set(WAITING)
                        await async_serial_writer(ser, "STATE_WAITING")

                    elif msg2 == "EMPTY":
                        print("[main] Received EMPTY (fallback) -> EMPTY")
                        await state_holder.set(EMPTY)

                    else:
                        # Double timeout: safe fallback
                        print("[main] Double timeout waiting for Arduino after STATE_FEEDING -> forcing WAITING (safe fallback)")
                        await state_holder.set(WAITING)
                        await async_serial_writer(ser, "STATE_WAITING")
                await asyncio.sleep(0.1)

            elif state == EMPTY:
                print("[main] state=EMPTY; waiting for 'FILLED' serial message from Arduino (or user).")
                try:
                    msg = await wait_for_serial(serial_queue, {"FILLED"}, timeout=None)
                    if msg == "FILLED":
                        print("[main] Received FILLED -> go to WAITING")
                        await state_holder.set(WAITING)
                        await async_serial_writer(ser, "STATE_WAITING")
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    print("[main] Exception while waiting for FILLED:", e)
                    await asyncio.sleep(0.5)
            else:
                print("[main] Unknown state:", state)
                await asyncio.sleep(0.1)

    finally:
        # cleanup
        stop_event.set()
        t.join(timeout=1.0)
        try:
            ser.close()
        except Exception:
            pass
        server_task.cancel()
        await asyncio.sleep(0.1)
        print("[main] shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted by user")

import asyncio
import serial
import time
from collections import deque
import random
import threading
from dataclasses import dataclass

# ---------------- CONFIG ----------------
PORT = "/dev/ttyUSB1"                # <<-- change to your Arduino port, e.g. "/dev/ttyUSB0" , "COM3" or "/dev/ttyUSB1"
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

# ---------------- Mock classifier ----------------
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

    async def push(self, label: str):
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
         - after purging older-than WINDOW_SECONDS,
         - the first (oldest) item in the remaining window is at least FIRST_IMAGE_MIN_AGE seconds old,
         - and >= DETECTION_THRESHOLD of the remaining items are TARGET_CLASS.
        """
        async with self.lock:
            now = time.time()
            cutoff_erase = now - FIRST_IMAGE_MIN_AGE

            # purge too-old frames
            while self.window and self.window[0][0] < cutoff_erase:
                print(f"now: {now}")
                print(f"self.window[0][0]: {self.window[0][0]}")
                self.window.popleft()

            if not self.window:
                print("vazio")
                return False

            # oldest_ts = self.window[0][0]
            # if (now - oldest_ts) < FIRST_IMAGE_MIN_AGE:
            #     print("imagem muito nova")
            #     return False

            cutoff = now - WINDOW_SECONDS
            total = sum(1 for time, _ in self.window if time < cutoff)
            if total <= 0:
                print("total vazio")
                return False
            positives = sum(1 for time, lbl in self.window if lbl == TARGET_CLASS and time < cutoff)
            ratio = positives / total
            print(f"ratio: {ratio}")
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
                                  detector: RollingDetector, state_holder: StateHolder):
    peer = writer.get_extra_info("peername")
    print(f"[img_server] connection from {peer}")
    try:
        while True:
            # read 4-byte size
            header = await reader.readexactly(4)
            img_size = int.from_bytes(header, "big")
            img_bytes = await reader.readexactly(img_size)
            print("Received image")

            # classify
            label = mock_classify_image(img_bytes)

            # only push if current state is DETECTING
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

async def start_image_server(detector: RollingDetector, state_holder: StateHolder):
    server = await asyncio.start_server(
        lambda r, w: handle_image_connection(r, w, detector, state_holder),
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
    state_holder = StateHolder(WAITING)

    # start image server
    server_task = asyncio.create_task(start_image_server(detector, state_holder))

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

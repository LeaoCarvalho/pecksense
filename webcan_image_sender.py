import cv2
import socket
import struct
import time

SERVER_IP = "127.0.0.1"   # change to your server
SERVER_PORT = 9000

# Connect to TCP server
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((SERVER_IP, SERVER_PORT))

# Webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not open webcam")
    exit()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Encode frame as JPEG
        success, encoded = cv2.imencode(".jpg", frame)
        if not success:
            continue

        data = encoded.tobytes()
        size = len(data)

        # Send size (4 bytes) + image data
        sock.sendall(struct.pack("!I", size) + data)

        # Wait 0.5 seconds
        # time.sleep(0.5)

except KeyboardInterrupt:
    pass

cap.release()
sock.close()

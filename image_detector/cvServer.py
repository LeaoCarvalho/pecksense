import asyncio
import cv2
import numpy as np
import os
import struct

# --- CONFIGURAÇÕES ---
SERVER_IP = "0.0.0.0"  # Escuta em todas as interfaces de rede
SERVER_PORT = 8888     # Porta para conexão

# Arquivos da IA
PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"
CONFIDENCE_THRESHOLD = 0.5
TARGET_CLASS = "bird"
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

class VisionSystem:
    def __init__(self):
        print("[SERVER] Carregando modelo IA...")
        if not os.path.exists(PROTOTXT) or not os.path.exists(MODEL):
            raise FileNotFoundError("Arquivos do modelo MobileNet não encontrados!")
        self.net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)
        self.cap = None

    def start_camera(self):
        for idx in [0, 1, -1]:
            print(f"[SERVER] Testando câmera {idx}...")
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    self.cap = cap
                    print(f"[SERVER] Câmera iniciada no índice {idx}")
                    return
                cap.release()
        print("[SERVER] ERRO: Nenhuma câmera encontrada!")

    def get_frame_and_classify(self):
        if self.cap is None: return None, "error"
        ret, frame = self.cap.read()
        if not ret: return None, "error"

        # IA
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        detected_label = "background"
        
        # Desenhar e Classificar
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > CONFIDENCE_THRESHOLD:
                idx = int(detections[0, 0, i, 1])
                label = CLASSES[idx]
                
                if label == TARGET_CLASS:
                    detected_label = TARGET_CLASS
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")
                    cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                    cv2.putText(frame, f"ALVO: {confidence:.2f}", (startX, startY-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame, detected_label

async def handle_client(reader, writer):
    addr = writer.get_extra_info('peername')
    print(f"[SERVER] Conexão aceita de {addr}")
    
    vision = VisionSystem()
    vision.start_camera()
    loop = asyncio.get_running_loop()

    try:
        while True:
            # 1. Captura e Processa (Executa em thread separada para não travar rede)
            frame, label = await loop.run_in_executor(None, vision.get_frame_and_classify)
            
            if frame is None:
                await asyncio.sleep(0.1)
                continue

            # 2. Comprime imagem para JPG (Qualidade 60 para ser rápido no WiFi)
            encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            img_bytes = buffer.tobytes()

            # 3. Empacota os dados para envio
            # Protocolo: [4 bytes Tam Label] [Label String] [4 bytes Tam Imagem] [Imagem Bytes]
            label_bytes = label.encode('utf-8')
            
            # Envia tamanho do label e o label
            writer.write(struct.pack('>I', len(label_bytes)))
            writer.write(label_bytes)
            
            # Envia tamanho da imagem e a imagem
            writer.write(struct.pack('>I', len(img_bytes)))
            writer.write(img_bytes)
            
            await writer.drain() # Garante que enviou
            await asyncio.sleep(0.01) # Cede tempo para CPU

    except ConnectionResetError:
        print(f"[SERVER] Cliente {addr} desconectou.")
    except Exception as e:
        print(f"[SERVER] Erro na conexão: {e}")
    finally:
        vision.cap.release()
        writer.close()
        await writer.wait_closed()
        print("[SERVER] Conexão encerrada, pronto para próxima.")

async def main():
    server = await asyncio.start_server(handle_client, SERVER_IP, SERVER_PORT)
    addrs = ', '.join(str(sock.getsockname()) for sock in server.sockets)
    print(f"[SERVER] Servidor de Visão rodando em {addrs}")
    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
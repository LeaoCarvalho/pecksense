import cv2
import numpy as np
import time

# --- CONFIGURAÇÕES GLOBAIS ---
PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"
CONFIDENCE_THRESHOLD = 0.5
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

COLOR_BIRD = (0, 255, 0)   # Verde
COLOR_OTHER = (0, 0, 255)  # Vermelho

def classify_frame(net, frame):
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
    net.setInput(blob)
    detections = net.forward()

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

            if class_name == "bird":
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

def encontrar_camera_disponivel():
    """Tenta encontrar uma câmera ativa testando índices comuns."""
    indices_para_testar = [0, 1, 2, -1]
    
    for idx in indices_para_testar:
        print(f"[DEBUG] Testando webcam no índice {idx}...")
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"[SUCESSO] Webcam encontrada no índice {idx}!")
                return cap
            else:
                cap.release()
    return None

def main():
    print("[INFO] Carregando modelo MobileNetSSD...")
    try:
        net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)
    except cv2.error:
        print("[ERRO] Arquivos do modelo não encontrados!")
        return

    print("[INFO] Procurando webcam...")
    cap = encontrar_camera_disponivel()
    
    if cap is None:
        print("\n[ERRO CRÍTICO] Nenhuma webcam foi encontrada!")
        return

    time.sleep(1.0) # Câmera esquentando
    bird_detected_start_time = None
    ALIMENTANDO = False

    print("[INFO] Sistema rodando. Pressione 'q' para sair.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # --- AQUI ESTÁ A MUDANÇA ---
        # Chamamos a função separada para fazer o trabalho pesado
        passaro_presente_neste_frame = classify_frame(net, frame)
        # ---------------------------

        # Lógica de validação temporal (anti-falso positivo)
        if passaro_presente_neste_frame:
            if bird_detected_start_time is None:
                bird_detected_start_time = time.time()
            
            tempo_presenca = time.time() - bird_detected_start_time
            
            # Desenha cronômetro na tela
            cv2.putText(frame, f"Presenca: {tempo_presenca:.1f}s", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if tempo_presenca > 2.0 and not ALIMENTANDO:
                print(f"[AÇÃO] ALIMENTAR! (Pássaro confirmado por {tempo_presenca:.1f}s)")
                cv2.putText(frame, "ALIMENTANDO...", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                ALIMENTANDO = True 
        else:
            bird_detected_start_time = None
            ALIMENTANDO = False

        cv2.imshow("Visao Computacional - Modular", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    print("[INFO] Encerrando...")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()